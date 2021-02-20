import threading


class EventsHandler:
    def __init__(self):
        self.eventTakeOffComplete = threading.Event()
        self.eventMissionComplete = threading.Event()
        self.eventBusy = threading.Event()
        self.eventLocationReached = threading.Event()
