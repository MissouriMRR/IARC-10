class Task:
    def __init__(self):
        self.remaining_time = 0

    def run_for_time(self, time):
        self.remaining_time -= time


    def get_remaining_time(self):
        return self.remaining_time
