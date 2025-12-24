"""
Hot-reload wrapper for controller.py
Automatically restarts when code changes are detected.
"""
import subprocess
import sys
import time
from pathlib import Path

try:
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler
except ImportError:
    print("Installing watchdog...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "watchdog"])
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler


SCRIPT_DIR = Path(__file__).parent
CONTROLLER_PATH = SCRIPT_DIR / "controller.py"


class ReloadHandler(FileSystemEventHandler):
    def __init__(self):
        self.process = None
        self.start_controller()
    
    def start_controller(self):
        if self.process:
            self.process.terminate()
            self.process.wait()
            print("\n>>> Restarting controller...\n")
        
        self.process = subprocess.Popen(
            [sys.executable, str(CONTROLLER_PATH)],
            cwd=str(SCRIPT_DIR)
        )
    
    def on_modified(self, event):
        if event.src_path.endswith("controller.py"):
            time.sleep(0.1)  # Brief delay for file to finish writing
            self.start_controller()
    
    def stop(self):
        if self.process:
            self.process.terminate()
            self.process.wait()


if __name__ == "__main__":
    print("=" * 50)
    print("HOT RELOAD MODE")
    print("Edit controller.py and it will auto-restart!")
    print("Press Ctrl+C to quit")
    print("=" * 50)
    print()
    
    handler = ReloadHandler()
    observer = Observer()
    observer.schedule(handler, str(SCRIPT_DIR), recursive=False)
    observer.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n>>> Shutting down...")
        handler.stop()
        observer.stop()
    
    observer.join()

