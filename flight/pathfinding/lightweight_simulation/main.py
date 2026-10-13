from GoToTask import GoToTask

my_task = GoToTask()

print(my_task.get_remaining_time())

my_task.run_for_time(3)

print(my_task.get_remaining_time())