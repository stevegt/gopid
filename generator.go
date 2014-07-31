// Copyright (c) 2014 Datacratic. All rights reserved.

package control

import (
	"math"
	"sync"
	"time"
)

// Task defines an interface for executing tasks.
type Task interface {
	Do()
}

// TaskFunc defines an helper type for using function literals as a Task.
type TaskFunc func()

// Do executes the function literal as a Task.
func (f TaskFunc) Do() {
	f()
}

// DefaultGeneratorSmoothingFactor contains the default weight of the exponential moving average that estimates the time taken by the handler.
var DefaultGeneratorSmoothingFactor = 0.80

// DefaultGeneratorSleepPrecision defines the default duration that will trigger a wait after each request.
var DefaultGeneratorSleepPrecision = 50 * time.Millisecond

// DefaultGeneratorSamplingPeriod defines the default approximate duration over which the QPS is sampled.
var DefaultGeneratorSamplingPeriod = time.Second

// Generator adapts itself to call an handler a fixed number of times per second.
// It operates by creating B concurrent batches that will call the handler N times repeatedly.
// After each call to the handler, it waits W seconds to space requests evently over time.
// To avoid sleeping for very small amount of time, those waits are grouped based on the supplied precision.
// Note that the system adjusts B and W based on its estimation of the time R taken by the handler.
// Therefore, if the variability of R is high, it may make it harder for the system to stabilize to a steady state.
type Generator struct {
	// Handler contains what gets executed periodically.
	Handler Task
	// QPS contains the number of calls that will be done per second.
	QPS float64
	// SmoothingFactor contains the weight of the exponential moving average that estimates the time taken by the handler.
	// Will use DefaultGeneratorSmoothingFactor if 0.
	SmoothingFactor float64
	// SleepPrecision contains the minimal duration that will trigger a wait after each request.
	// Will use DefaultGeneratorSleepPrecision if 0.
	SleepPrecision time.Duration
	// SamplingPeriod contains the approximate duration over which the QPS is sampled.
	// Will use DefaultGeneratorSamplingPeriod if 0.
	SamplingPeriod time.Duration
}

// Start begins calling the handler at the requested frequency.
func (generator *Generator) Start() {
	if generator.Handler == nil {
		panic("generator requires an handler")
	}

	go generator.run()
}

func (generator *Generator) evaluate(r float64) (b int, n int, w float64) {
	sampling := generator.SamplingPeriod
	if sampling == 0 {
		sampling = DefaultGeneratorSamplingPeriod
	}

	s := sampling.Seconds()

	// sampling period S=(R+W)*N
	// with maximum QPS per B is Q=1/R and B = QPS/Q
	// thus, because QPS=B/(R+W)
	// we end up with both W=B/QPS-R and N=S/(R+W)

	qps := generator.QPS
	if r > 0 {
		b = int(math.Ceil(qps * r))
	} else {
		b = 1
	}

	w = float64(b)/qps - r
	n = int(math.Ceil(s / (r + w)))
	return
}

func (generator *Generator) run() {
	precision := generator.SleepPrecision
	if precision == 0 {
		precision = DefaultGeneratorSleepPrecision
	}

	type batch struct {
		requests int
		duration time.Duration
		passback interface{}
	}

	results := make(chan batch)

	pool := sync.Pool{
		New: func() interface{} {
			batches := make(chan batch)

			// starts a batch that executes tasks and waits N times
			go func() {
				for item := range batches {
					t0 := time.Now()
					dt := time.Duration(0)

					for i := 0; i != item.requests; i++ {
						generator.Handler.Do()
						if dt += item.duration; dt >= precision {
							time.Sleep(dt)
							dt = 0
						}
					}

					time.Sleep(dt)
					item.duration = time.Since(t0)
					results <- item
				}
			}()

			return batches
		},
	}

	// perform a quick approximation with a first batch of 1 request
	queue := pool.Get().(chan batch)
	queue <- batch{
		requests: 1,
		duration: 0,
		passback: queue,
	}

	first := <-results
	pool.Put(first.passback)

	// set initial values
	r := first.duration.Seconds()
	batches, n, w := generator.evaluate(r)
	working := 0

	ticks := time.NewTicker(time.Second)
	count := 0
	total := batch{}

	smoothing := generator.SmoothingFactor
	if smoothing == 0 {
		smoothing = DefaultGeneratorSmoothingFactor
	}

	for {
		// start as many concurrent batches as it is required
		for working < batches {
			working++
			queue := pool.Get().(chan batch)
			queue <- batch{
				requests: n,
				duration: time.Duration(1e9*w) * time.Nanosecond,
				passback: queue,
			}
		}

		select {
		case item := <-results:
			count++
			total.requests += item.requests
			total.duration += item.duration

			working--
			pool.Put(item.passback)

		case <-ticks.C:
			if count == 0 {
				break
			}

			m := smoothing
			r = m*r + (1.0-m)*(total.duration.Seconds()/float64(total.requests)-w)
			batches, n, w = generator.evaluate(r)

			count = 0
			total.requests = 0
			total.duration = 0
		}
	}
}
