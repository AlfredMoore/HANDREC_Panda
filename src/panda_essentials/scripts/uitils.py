import rospy as ros

class ModuleTimer:
    def __init__(self, module_name: str):
        self.name = module_name
        self.start_time: ros.Time
        self.end_time: ros.Time

    def start(self):
        self.start_time = ros.Time.now()

    def end(self):
        self.end_time = ros.Time.now()
    
    def get_time_consumption(self):
        duration: ros.Duration = self.end_time - self.start_time
        ros.loginfo(f"{self.name}: {duration.to_sec()+duration.to_nsec()/1000} seconds")