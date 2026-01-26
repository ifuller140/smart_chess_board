#!/usr/bin/env python3
"""
Base class for hardware tests.

All hardware tests inherit from HardwareTest to get consistent
behavior with display feedback and clock button input.
"""

import time
from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional, Callable, List, Tuple


class TestResult(Enum):
    """Result of a test step or overall test."""
    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    SKIPPED = "skipped"


class TestStep:
    """Represents a single step in a hardware test."""
    
    def __init__(
        self,
        name: str,
        display_text: str,
        action: Callable[[], bool],
        wait_for_input: bool = False,
        input_type: str = "clock",  # "clock", "x_limit", "y_limit"
        timeout_seconds: float = 30.0,
        success_message: str = "OK",
        failure_message: str = "FAIL"
    ):
        """
        Initialize a test step.
        
        Args:
            name: Human-readable step name
            display_text: Text to show on clock display (max 8 chars)
            action: Function to execute for this step (returns True on success)
            wait_for_input: Whether to wait for user input before proceeding
            input_type: Type of input to wait for
            timeout_seconds: Maximum time to wait for input
            success_message: Message to display on success
            failure_message: Message to display on failure
        """
        self.name = name
        self.display_text = display_text[:8]  # Limit to 8 chars for display
        self.action = action
        self.wait_for_input = wait_for_input
        self.input_type = input_type
        self.timeout_seconds = timeout_seconds
        self.success_message = success_message[:8]
        self.failure_message = failure_message[:8]
        self.result = TestResult.PENDING
        self.error_message: Optional[str] = None


class HardwareTest(ABC):
    """
    Abstract base class for hardware tests.
    
    Subclasses must implement:
        - name: Test category name
        - description: What this test validates
        - get_steps(): Returns list of TestStep objects
    """
    
    def __init__(self, gpio_interface=None, display_interface=None):
        """
        Initialize the hardware test.
        
        Args:
            gpio_interface: GPIO control interface (or mock for testing)
            display_interface: Clock display interface (or mock for testing)
        """
        self.gpio = gpio_interface
        self.display = display_interface
        self.steps: List[TestStep] = []
        self.current_step_index = 0
        self.overall_result = TestResult.PENDING
        
        # Input state
        self._clock_pressed = False
        self._x_limit_pressed = False
        self._y_limit_pressed = False
    
    @property
    @abstractmethod
    def name(self) -> str:
        """Test category name (e.g., 'Gantry', 'Servo', 'Camera')."""
        pass
    
    @property
    @abstractmethod
    def description(self) -> str:
        """Brief description of what this test validates."""
        pass
    
    @abstractmethod
    def get_steps(self) -> List[TestStep]:
        """Return list of test steps to execute."""
        pass
    
    def setup(self) -> bool:
        """
        Setup before running tests. Override in subclass if needed.
        
        Returns:
            True if setup successful, False otherwise
        """
        return True
    
    def teardown(self):
        """Cleanup after tests. Override in subclass if needed."""
        pass
    
    def show_display(self, text: str):
        """Show text on the clock display."""
        if self.display:
            self.display.show_text(text)
        else:
            print(f"[DISPLAY] {text}")
    
    def wait_for_clock_button(self, timeout: float = 30.0) -> bool:
        """
        Wait for clock button press.
        
        Args:
            timeout: Maximum seconds to wait
            
        Returns:
            True if button pressed within timeout, False otherwise
            
        Raises:
            RuntimeError: If GPIO interface is not available
        """
        if self.gpio is None:
            raise RuntimeError("GPIO interface required - hardware must be connected")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.gpio.read_clock_button():
                # Debounce
                time.sleep(0.05)
                if self.gpio.read_clock_button():
                    # Wait for release
                    while self.gpio.read_clock_button():
                        time.sleep(0.01)
                    return True
            time.sleep(0.01)
        return False
    
    def wait_for_limit_switch(self, switch: str, timeout: float = 30.0) -> bool:
        """
        Wait for a limit switch to be triggered.
        
        Args:
            switch: "x" or "y"
            timeout: Maximum seconds to wait
            
        Returns:
            True if switch triggered within timeout, False otherwise
            
        Raises:
            RuntimeError: If GPIO interface is not available
        """
        if self.gpio is None:
            raise RuntimeError("GPIO interface required - hardware must be connected")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if switch == "x" and self.gpio.read_x_limit():
                return True
            elif switch == "y" and self.gpio.read_y_limit():
                return True
            time.sleep(0.01)
        return False
    
    def run(self, verbose: bool = True) -> TestResult:
        """
        Run all test steps.
        
        Args:
            verbose: Print detailed output
            
        Returns:
            Overall test result
        """
        self.steps = self.get_steps()
        self.overall_result = TestResult.RUNNING
        
        if verbose:
            print(f"\n{'='*60}")
            print(f"  {self.name} Test")
            print(f"  {self.description}")
            print(f"{'='*60}\n")
        
        # Setup
        if not self.setup():
            self.overall_result = TestResult.FAILED
            self.show_display("SETUP ER")
            if verbose:
                print("[FAILED] Setup failed")
            return self.overall_result
        
        # Run each step
        try:
            for i, step in enumerate(self.steps):
                self.current_step_index = i
                step.result = TestResult.RUNNING
                
                if verbose:
                    print(f"\nStep {i+1}/{len(self.steps)}: {step.name}")
                
                # Show step on display
                self.show_display(step.display_text)
                
                # Wait for input if required
                if step.wait_for_input:
                    if verbose:
                        print(f"  Waiting for {step.input_type} input...")
                    
                    if step.input_type == "clock":
                        input_received = self.wait_for_clock_button(step.timeout_seconds)
                    elif step.input_type == "x_limit":
                        input_received = self.wait_for_limit_switch("x", step.timeout_seconds)
                    elif step.input_type == "y_limit":
                        input_received = self.wait_for_limit_switch("y", step.timeout_seconds)
                    else:
                        input_received = False
                    
                    if not input_received:
                        step.result = TestResult.FAILED
                        step.error_message = f"Timeout waiting for {step.input_type}"
                        self.show_display("TIMEOUT")
                        if verbose:
                            print(f"  [FAILED] {step.error_message}")
                        self.overall_result = TestResult.FAILED
                        break
                
                # Execute action
                try:
                    success = step.action()
                except Exception as e:
                    success = False
                    step.error_message = str(e)
                
                if success:
                    step.result = TestResult.PASSED
                    self.show_display(step.success_message)
                    if verbose:
                        print(f"  [PASSED] {step.success_message}")
                else:
                    step.result = TestResult.FAILED
                    self.show_display(step.failure_message)
                    if verbose:
                        msg = step.error_message or step.failure_message
                        print(f"  [FAILED] {msg}")
                    self.overall_result = TestResult.FAILED
                    break
                
                # Small delay between steps
                time.sleep(0.3)
            
            # All steps passed
            if self.overall_result == TestResult.RUNNING:
                self.overall_result = TestResult.PASSED
                self.show_display("PASS")
                if verbose:
                    print(f"\n{'='*60}")
                    print(f"  {self.name} Test: PASSED")
                    print(f"{'='*60}\n")
            else:
                self.show_display("FAIL")
                if verbose:
                    print(f"\n{'='*60}")
                    print(f"  {self.name} Test: FAILED")
                    print(f"{'='*60}\n")
                    
        finally:
            self.teardown()
        
        return self.overall_result
    
    def get_summary(self) -> str:
        """Get a summary of test results."""
        lines = [
            f"{self.name} Test Summary",
            "-" * 40,
        ]
        
        for i, step in enumerate(self.steps):
            status = step.result.value.upper()
            lines.append(f"  {i+1}. {step.name}: {status}")
            if step.error_message:
                lines.append(f"      Error: {step.error_message}")
        
        lines.append("-" * 40)
        lines.append(f"Overall: {self.overall_result.value.upper()}")
        
        return "\n".join(lines)
