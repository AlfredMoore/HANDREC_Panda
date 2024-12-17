import rospy as ros
from functools import wraps
from geometry_msgs.msg import TransformStamped, Pose

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


def validate_params(obj) -> bool:
    # Get public attributes and their values using vars()
    invalid_list = []
    for attr, value in vars(obj).items():
        if not attr.startswith('_'):
            if value is None:
                invalid_list.append(attr)
    
    if invalid_list:
        ros.logerror(f"Invalid ROS parameters: {', '.join(invalid_list)}")
        return False
    else:
        return True
    
def transformstamped_to_pose(transform: TransformStamped) -> Pose:
    pose = Pose()
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    pose.orientation.x = transform.transform.rotation.x
    pose.orientation.y = transform.transform.rotation.y
    pose.orientation.z = transform.transform.rotation.z
    pose.orientation.w = transform.transform.rotation.w
    return pose