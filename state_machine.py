class State:
    def __init__(self, action_func, uncond_next = None, true_next = None, false_next = None):
        self.action_func = action_func
        self.uncond_next = uncond_next
        self.true_next = true_next
        self.false_next = false_next
    
    def set_true(self, state):
        self.true_next = state
    
    def set_false(self, state):
        self.false_next = state
        
    def execute(self, *args):
        result = self.action_func(*args)
    
        if result == True and self.true_next != None: self.true_next.execute()
        if result == False and self.false_next != None: self.false_next.execute()
        if uncond_next != None: self.uncond_next.execute()