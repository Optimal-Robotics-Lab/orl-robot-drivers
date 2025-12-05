import time
from datetime import datetime
import sys

def main():
    print(f"--- Clock Synchronization Check ---")
    print(f"Run this on both HOST and ROBOT simultaneously.")
    print(f"Watch to see if the lines print at the exact same moment.")
    print(f"-----------------------------------")
    print(f"{'Human Time':<25} | {'Unix Timestamp':<20}")

    try:
        while True:
            # Calculate time to sleep to hit the next exact second
            # This aligns the print output visually on both screens
            now = time.time()
            sleep_needed = 1.0 - (now % 1.0)
            
            # Sleep slightly less to ensure we don't miss the 0.0 rollover due to scheduler
            time.sleep(sleep_needed)
            
            # Get the actual time immediately after waking up
            current_time = time.time()
            local_dt = datetime.fromtimestamp(current_time)
            
            # Format: HH:MM:SS.microseconds
            human_time = local_dt.strftime('%H:%M:%S.%f')
            
            # Print the time. 
            # If synced, these lines should appear on both screens simultaneously.
            # Compare the decimal places of the Unix Timestamp.
            print(f"{human_time} | {current_time:.6f}")
            
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)

if __name__ == "__main__":
    main()