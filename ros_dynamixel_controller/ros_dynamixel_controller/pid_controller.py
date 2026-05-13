class LowPassFilter:
    """First-order low-pass filter (exponential moving average)."""
    
    def __init__(self, alpha=0.1):
        """
        Initialize low-pass filter.
        
        Args:
            alpha: Filter coefficient (0-1). Lower values = more smoothing.
                   alpha = dt / (dt + RC) where RC = 1/(2πfc)
        """
        self.alpha = alpha
        self.filtered_value = 0.0
    
    def filter(self, raw_value):
        self.filtered_value = self.alpha * raw_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    
    def reset(self):
        """Reset filter state."""
        self.filtered_value = 0.0

class PIDController:
    def __init__(self, kp, ki, kd):
        """
        Initialize PID controller with anti-windup protection and low-pass filters.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = 10.0
        self.wind_down_coeff = 1.0
        
        # State variables
        self.integral = 0.0
        self.previous_error = 0.0
        
        # Filter states
        self.error_filter = LowPassFilter(alpha=1.0)
        self.derivative_filter = LowPassFilter(alpha=1.0)
        self.output_filter = LowPassFilter(alpha=1.0)
        
    def advance(self, current_error, dt=0.1, velocity=0.0):
        """
        Advance the PID controller with the current error.
        
        Args:
            current_error: Current error value (float) - usually setpoint - process_value
            dt: Time step (seconds)
            
        Returns:
            float: PID control output
        """
        # Apply wind-down coefficient to integral term before updating
        self.integral *= self.wind_down_coeff

        filtered_error = self.error_filter.filter(current_error)
        
        # Proportional term (no filtering needed - instant response)
        p_term = self.kp * filtered_error
        
        # Integral term with clamping to prevent windup
        integral_increment = self.ki * filtered_error * dt
        self.integral += integral_increment
        
        # Clamp integral term to max_integral range
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral
        
        i_term = self.integral
        
        # Derivative term with low-pass filtering
        raw_derivative = (filtered_error - self.previous_error) / dt
        
        # Apply first-order low-pass filter to derivative
        filtered_derivative = self.derivative_filter.filter(raw_derivative)
        
        # d_term = self.kd * filtered_derivative
        d_term = self.kd * velocity
        
        # Calculate raw output
        raw_output = p_term + i_term - d_term
        
        # Apply low-pass filter to output for smooth control signals
        filtered_output = self.output_filter.filter(raw_output)
        
        # Store previous error for next derivative calculation
        self.previous_error = filtered_error
        
        return filtered_output
    
    def reset(self):
        """Reset the controller state."""
        self.integral = 0.0
        self.previous_error = 0.0
        self.error_filter.reset()
        self.derivative_filter.reset()
        self.output_filter.reset()