// Copyright (c) 2014 Datacratic. All rights reserved.

package control

import (
	"fmt"
	"math"
	"time"
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
	Output    float64
	MinOutput float64
	MaxOutput float64

	delta       float64
	integral    float64
	timestamp   time.Time
	prev_target float64
	prev_delta  float64
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
	fmt.Printf("pid input %.2f target %.2f output %.2f kp %.4f ki %.4f kd %.4f delta %.3f step %.3f integral %.3f derivative %.3f\n", value, c.Target, output, c.Kp, c.Ki, c.Kd, delta, step, c.integral, derivative)
	return
}
