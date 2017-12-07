class stateaction:
    def __init__(self,acc,tiempo):
        self.tiempo = tiempo
        self.action = acc


class state:

    def __init__(self, action, entry, exi):
        self.entryaction = entry
        self.action = action
        self.exitaction = exi

    def getentryaction(self):
        return self.entryaction

    def getaction(self):
        return self.action

    def getexitaction(self):
        return self.exitaction

    def gettransitions(self, smachine):
        ts = list([])
        for t in smachine.transitions:
            if (t.afrom == self):
                ts.append(t)
        return ts




class transition:
    def __init__(self, trigger, state, afrom,action = None):
        self.trigger = trigger
        self.targetstate = state
        self.action = action
        self.afrom = afrom


    def gettrigger(self):
        return self.trigger

    def getaction(self):
        return self.action

    def getstate(self):
        return self.state

    def gettargetstate(self):
        return self.targetstate

    def istriggered(self,pc):
        return self.trigger(pc)



class statemachine:

    def __init__(self,states, initial,kinematicfrom,transitions = list([])):
        self.states = states
        self.current = initial
        self.transitions = transitions
        self.pc = kinematicfrom

    def update(self):

        triggeredtransition = None

        for transition in self.current.gettransitions(self):
            if transition.istriggered(self.pc):
                triggeredtransition = transition
                break

        if triggeredtransition != None:
            targetstate = triggeredtransition.gettargetstate()
            x = self.current.getexitaction()
            y = self.current.getaction()
            z = targetstate.getentryaction()
            actions = [x,y,z]

            self.current = targetstate
            if x == None and z == None:
                return [y]
            return actions

        else:
            return [self.current.getaction()]









        
    
