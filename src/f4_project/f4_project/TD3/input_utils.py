import sys
import tty
import termios
import threading
import time
import numpy as np

class KeyboardController:
    def __init__(self, action_dim=2):
        self.action_dim = action_dim
        self.action = np.zeros(action_dim)
        self.running = True
        self.thread = threading.Thread(target=self._input_loop)
        self.thread.daemon = True
        self.thread.start()
        print("\nKeyboard Control Enabled: W/S for X, A/D for Y, Q to quit control, P to pause.")

    def _getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def _input_loop(self):
        while self.running:
            try:
                # Simple non-blocking check would be better but standard python limited.
                # Here we block for 1 char. This might delay loop if no input.
                # Better approach: Read key, set state, decay state or keep it until key up (hard in terminal).
                # Terminal approach: Key press toggles direction or sets it for a duration.
                # Let's use: Press sets value, it auto-decays or needs counter-press?
                # Easiest for continuous control: Press increments/sets value, user must tap to sustain or we zero it out?
                # Let's assume user holds key -> repeats chars.
                
                # We'll set a target velocity and decay it to zero if no input received quickly?
                # Actually, blocking read makes it hard to decay.
                # Let's try select to make it non-blocking.
                
                import select
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = self._getch()
                    if key == 'w':
                        self.action[0] = 1.0
                    elif key == 's':
                        self.action[0] = -1.0
                    elif key == 'a':
                        self.action[1] = 1.0 # Left positive Y
                    elif key == 'd':
                        self.action[1] = -1.0 # Right negative Y
                    elif key == ' ':
                         self.action = np.zeros(self.action_dim)
                    elif key == 'q':
                        self.running = False
                    
                    # Decaying logic: 
                    # If we don't zero it, it acts like latching throttle.
                    # If we zero it immediately, it's very jerky.
                    # Let's act like keys are "nudges" or set a short-lived state.
                    # Since we verify this in a thread, let's set a timestamp and clear it in the getter if old.
                    self.last_press_time = time.time()
                else:
                    # No key pressed
                    pass
            except:
                self.running = False

    def get_action(self):
        # Decay action if no recent press (e.g. 0.2s)
        if hasattr(self, 'last_press_time') and time.time() - self.last_press_time > 0.2:
            self.action = np.zeros(self.action_dim)
        
        return self.action.copy()

    def stop(self):
        self.running = False
