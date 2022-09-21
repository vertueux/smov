class Events:
    def __init__(self):
        self.allow_process_event = True
    
    def allows(self):
        return self.allow_process_event
