import rospy as ros
from functools import wraps


def time_consumption(enabled=True):
    """
    A decorator to measure the time consumption of a function.
    Includes the class name if used within a class.
    If `enabled` is False, the timer is disabled.
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            if not enabled:  # Check if the timer is disabled
                return func(*args, **kwargs)  # Call the function without timing

            # Get the class name if it's a bound method
            class_name = None
            if args and hasattr(args[0], '__class__'):
                class_name = args[0].__class__.__name__

            start_time = ros.Time.now()  # Record the start time
            result = func(*args, **kwargs)  # Call the original function
            end_time = ros.Time.now()  # Record the end time
            duration: ros.Duration = end_time - start_time

            # Include class name in the output if available
            if class_name:
                ros.loginfo(f"{class_name}.{func.__name__}: {duration.to_sec()+duration.to_nsec()/1000} seconds")
            else:
                ros.loginfo(f"{func.__name__}: {duration.to_sec()+duration.to_nsec()/1000} seconds")
            return result

        return wrapper

    return decorator