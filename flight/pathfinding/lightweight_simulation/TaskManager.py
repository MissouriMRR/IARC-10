from Task import Task

class TaskManager:
    def __init__(self):
        self.tasks = []
        self.index = 0

    def addTask(self, task:"Task"):
        self.tasks.append(task)

    def step(self):
        value =  self.tasks[self.index]

