// Copyright (c) 2014 Datacratic. All rights reserved.

package gopid

import (
	"math"
	"sync/atomic"
	"testing"
	"time"
)

func TestGenerator(t *testing.T) {
	var count int32

	n := 1000.0
	k := 2.0

	results := make(chan time.Time)
	h := func() {
		// some payload
		time.Sleep(100 * time.Millisecond)

		c := atomic.AddInt32(&count, 1)
		if c == int32(n*k) {
			results <- time.Now()
		}
	}

	g := Generator{
		QPS:     n,
		Handler: TaskFunc(h),
	}

	t0 := time.Now()
	g.Start()
	result := <-results

	dt := result.Sub(t0).Seconds()
	qps := n * k / dt

	if math.Abs(n-qps) > n*0.1 {
		t.Fatalf("was expecting qps=%f after %f seconds and got qps=%f", n, k, qps)
	}
}
