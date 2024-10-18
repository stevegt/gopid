// Copyright (c) 2014 Datacratic. All rights reserved.

package control

import (
	"fmt"
	"math"
	"reflect"
	"time"

	"gonum.org/v1/gonum/stat"
)

// PID defines a generic PID controller.
type PID struct {
	// Kp contains the proportional gain.
	Kp float64
	// Ki contains the integral gain.
	Ki float64
	// Kd contains the derivative gain.
	Kd float64
	// Target contains the current setpoint.
	Target float64
	// Output contains the current value of the controller.
	// This value can be set before the first iteration to set initial conditions.
	Output     float64
	MinOutput  float64
	MaxOutput  float64
	MeanOutput float64
	MeanLength int

	delta       float64
	integral    float64
	timestamp   time.Time
	prev_target float64
	prev_delta  float64
	outputs     []float64
}

// Update performs an iteration of the PID controller with a variable dt based on the system time.
func (c *PID) Update(value float64) (output float64) {
	now := time.Now()

	if c.timestamp.IsZero() {
		output = c.Output
	} else {
		output = c.Step(value, now.Sub(c.timestamp))
	}

	c.timestamp = now
	return
}

// Step performs an iteration of the PID controller.
func (c *PID) Step(value float64, dt time.Duration) (output float64) {
	if dt == 0 {
		output = c.Output
		return
	}
	delta := c.Target - value
	step := dt.Seconds()
	if c.Target != c.prev_target {
		// integral is no longer valid if target has changed
		c.integral = 0
	}
	c.prev_target = c.Target
	if (delta < 0 && c.Output > c.MinOutput) || (delta > 0 && c.Output < c.MaxOutput) {
		// We only increase the integral if we're not already pegged
		// at min or max output. If we're properly tuned but pegged,
		// then our system is underpowered. In that case, it doesn't
		// do any good to continue to accumulate the integral; that
		// will just cause overshoot.
		c.integral += delta * step
	}
	derivative := (delta - c.delta) / step
	c.delta = delta
	output = c.Kp*delta + c.Ki*c.integral + c.Kd*derivative
	output = math.Min(c.MaxOutput, math.Max(c.MinOutput, output))
	c.Output = output
	// create MeanOutput XXX move all this to PIDHistory
	c.outputs = append(c.outputs, output)
	if len(c.outputs) >= c.MeanLength {
		c.outputs = c.outputs[len(c.outputs)-c.MeanLength:]
	}
	c.MeanOutput = stat.Mean(c.outputs, nil)
	fmt.Printf("pid input %.2f target %.2f output %.2f mean %.2f kp %.4f ki %.6f kd %.4f delta %.3f step %.3f integral %.3f derivative %.3f\n", value, c.Target, output, c.MeanOutput, c.Kp, c.Ki, c.Kd, delta, step, c.integral, derivative)
	return
}

type PIDHistory struct {
	PIDs          []PID
	MaxHistLength int
}

func NewPIDHistory(maxlen int) (hist PIDHistory) {
	hist = PIDHistory{MaxHistLength: maxlen}
	return
}

func (hist *PIDHistory) Append(pid PID) {
	hist.PIDs = append(hist.PIDs, pid)
	if len(hist.PIDs) >= hist.MaxHistLength {
		hist.PIDs = hist.PIDs[len(hist.PIDs)-hist.MaxHistLength:]
	}
}

func (hist *PIDHistory) List(key string) (list []float64) {
	for _, pid := range hist.PIDs {
		// get struct value by key name
		// https://stackoverflow.com/questions/18930910/access-struct-property-by-name
		r := reflect.ValueOf(pid)
		val := reflect.Indirect(r).FieldByName(key)
		if !val.IsValid() {
			fmt.Printf("invalid value of %s: %#v\n", key, hist)
		}
		list = append(list, val.Float())
	}
	return
}

func (hist *PIDHistory) Tune(pid PID) {
	// add pid to history
	hist.Append(pid)

	// calculate averages
	aout := stat.Mean(hist.List("Output"), nil)
	aintegral := stat.Mean(hist.List("integral"), nil)

	// calculate new K values
	Ki := aout / aintegral

	// just print recommendations for now
	fmt.Printf("recommended Ki %f\n", Ki)
}
