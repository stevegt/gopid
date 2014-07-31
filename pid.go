// Copyright (c) 2014 Datacratic. All rights reserved.

package control

import "time"

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
	Output float64

	delta     float64
	integral  float64
	timestamp time.Time
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
	c.integral += delta * step
	derivative := (delta - c.delta) / step
	c.delta = delta
	output = c.Kp*delta + c.Ki*c.integral + c.Kd*derivative
	c.Output = output
	return
}
