"""
Copyright 2015 Paul T. Grogan, Massachusetts Institute of Technology

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Simulator class.
"""

from .observable import Observable

class Simulator(Observable):
    def __init__(self, entities=None, initTime=0, timeStep=1, maxTime=10):
        """
        @param entities: the set of entities
        @type entities: L{list}
        @param initTime: the initial simulation time
        @type initTime: L{float}
        @param timeStep: the simulation time step
        @type timeStep: L{float}
        @param maxTime: the maximum simulation time
        @type maxTime: L{float}
        """
        Observable.__init__(self)
        if entities is None:
            self.entities = []
        else:
            self.entities = entities
        self.timeStep = timeStep
        self.initTime = initTime
        self.maxTime = maxTime
    
    def entity(self, name):
        return next((e for e in self.entities if e.name == name), None)
    
    def init(self):
        self.time = self.initTime
        for entity in self.entities:
            entity.init(self)
        self.trigger('init', self.time)
    
    def advance(self):
        if not self.isComplete():
            for entity in self.entities:
                # print "simulator entity:", entity
                entity.tick(self)

            for entity in self.entities:
                # print [[g.cash for g in f.getFederates()] for f in entity.federations]
                entity.tock()
                # print [[g.cash for g in f.getFederates()] for f in entity.federations]
            self.time += self.timeStep
            if self.time%10 == 0:
                print "time: ", self.time

            self.trigger('advance', self.time)
            if self.isComplete():
                self.trigger('complete', self.time)
    
    def execute(self):
        self.init()
        while not self.isComplete():
            self.advance()

        fedlist = []
        # for e in self.entities:
        #     print "entity: ", e
        #     for f in e.federations:
        #         print "federation: ", f
        #         for g in f.getFederates():
        #             print "federate:", g
        #             fedlist.append(g)

        # for fed in fedlist:
        #     print "Federate Name:", fed
        #     fed.showCostRewards()
    
    def isComplete(self):
        return (self.time >= self.maxTime
                if self.maxTime is not None else False)