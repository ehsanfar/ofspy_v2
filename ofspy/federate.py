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
Federate class.
"""
import numpy as np
import re
import logging

from .controller import Controller
from .operations import Operations
# from .algorithms import list2dict

class Federate(Controller):
    def __init__(self, name=None, initialCash=0, elements=None,
                 contracts=None, operations=Operations()):
        """
        @param name: the name of this federate
        @type name: L{str}
        @param initialCash: the initial cash for this federate
        @type initialCash: L{float}
        @param elements: the elements controlled by this federate
        @type elements: L{list}
        @param contracts: the contracts owned by this federate
        @type contracts: L{list}
        @param operations: the operations model of this federate
        @type operations: L{Operations}
        """
        Controller.__init__(self, name=name)
        self.initialCash = initialCash
        self.cash = self.initialCash
        if elements is None:
            self._initElements = []
        else:
            self._initElements = elements[:]
        self.elements = self._initElements
        if contracts is None:
            self._initContracts = []
        else:
            self._initContracts = contracts[:]
        self.contracts = self._initContracts
        self.operations = operations
        self.costDic = {}
        # self.contractSignals = {}
        # self.demandSignals = {}
        self.thirdContract = {}
        self.receivedDemand = {}
        self.issuedDemand = {}
        self.costHistory = {}
        self.name = name

    def getElements(self):
        """
        Gets the elements controlled by this controller.
        @return L{list}
        """
        return self.elements[:]
    
    def getFederates(self):
        """
        Gets the federates controlled by this controller.
        @return L{list}
        """
        return [self]
    
    def getContracts(self):
        """
        Gets the contracts controlled by this controller.
        @return L{list}
        """
        return self.contracts[:]
    
    def design(self, element):
        """
        Designs an element for this federate.
        @param element: the element to design
        @type element: L{Element}
        @return L{bool}
        """
        if element.getContentsSize() > element.capacity:
            logging.warning('{0} contents exceeds capacity.'
                         .format(element.name))
        elif element.getDesignCost() > self.cash:
            logging.warning('{0} design costs exceeds cash.'
                         .format(element.name))
        else:
            self.elements.append(element)
            cost = element.getDesignCost()
            self.cash -= cost
            logging.info('{0} designed {1} for {2}'
                        .format(self.name, element.name, cost))
            self.trigger('design', self, element, cost)
            return True
        return False
    
    def commission(self, element, location, context):
        """
        Commissions an element at a location.
        @param element: the element to commission
        @type element: L{Element}
        @param location: the location at which to commission
        @type location: L{Location}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        if element not in self.getElements():
            logging.warning('{0} does not control {1}.'
                        .format(self.name, element.name))
        elif element.getCommissionCost(location, context) > self.cash:
            logging.warning('{0} commission cost exceeds cash.'
                         .format(element.name))
        elif element.commission(location, context):
            logging.info('{0} commissioned {1} for {2}.'
                        .format(self.name, element.name,
                                element.getCommissionCost(location, context)))
            cost = element.getCommissionCost(location, context)
            self.cash -= cost
            self.trigger('commission', self, element, location, cost)
            return True
        else:
            logging.warning('{0} could not commission {1}.'
                         .format(self.name, element.name))
        return False
    
    def decommission(self, element):
        """
        Decommissions an element.
        @param element: the element to decommission
        @type element: L{Element}
        @return: L{bool}
        """
        if element not in self.getElements():
            logging.info('{0} could not decommission {1}.'.format(
                self.name, element.name))
        else:
            self.elements.remove(element)
            # self.cash += element.getDecommissionValue()
            logging.info('{0} decommissioned {1} for {2}.'.format(
                self.name, element.name, element.getDecommissionValue()))
            self.trigger('decommission', self, element)
            return True
        return False
    
    def init(self, sim):
        """
        Initializes this federate in a simulation.
        @param sim: the simulator
        """
        super(Federate, self).init(sim)
        self.cash = self.initialCash
        self.elements = self._initElements[:]
        for element in self.elements:
            element.init(sim)
        self.contracts = self._initContracts[:]
        for contract in self.contracts:
            contract.init(sim)
    
    def tick(self, sim):
        """
        Ticks this federate in a simulation.
        @param sim: the simulator
        """
        super(Federate, self).tick(sim)
        for element in self.elements:
            element.tick(sim)
        for contract in self.contracts:
            contract.tick(sim)
        # print "Tick cash: ", self.cash
    
    def tock(self):
        """
        Tocks this federate in a simulation.
        """
        super(Federate, self).tock()
        for element in self.elements:
            element.tock()
        for contract in self.contracts:
            contract.tock()

        # print "Tock cash: ", self.cash

    def setCost(self, protocol, cost):
        self.costDic[protocol] = cost

    def getCost(self, protocol, federate=None):

        key = '{}-{}'.format(federate, protocol)
        return self.costDic[protocol] if key not in self.costDic else self.costDic[key]
        # name_dic = {'P1': 300, 'P2': 600, 'P3': 900}
        # c = 200*np.round(10*np.random.normal()) +

        # mutual_cost = []
        # for k, v in self.receivedDemand.items():
        #     if protocol not in k or federate not in k:
        #         continue
        #
        #     g = re.search(r'.+_(\w+)_(\d+)', k).groups()
        #     cost = int(g[1])
        #     mutual_cost.append()

        # if protocol not in self.costHistory:
        #     self.costHistory[protocol] = []
        #
        # self.costHistory[protocol].append(c)
        # return name_dic[self.name]
        # return

    # def addContractSignal(self, issuer, protocol, cost):
    #     """
    #     :param cType: contract type
    #     :param issuer: the issuer of the contract
    #     """
    #     k = '{0}_{1}_{2}'.format(issuer, protocol, cost)
    #     self.contractSignals[k] = self.contractSignals[k]+1 if k in self.contractSignals else 1
    #
    # def addDemandSignal(self, issuer, protocol, cost):
    #     """
    #     :param cType: contract type
    #     :param issuer: the issuer of the contract
    #     """
    #     k = '{0}_{1}_{2}'.format(issuer, protocol, cost)
    #     self.demandSignals[k] = self.demandSignals[k]+1 if k in self.demandSignals else 1


    def addThirdContract(self, sender, protocol, cost):
        """
        :param cType: contract type
        :param issuer: the issuer of the contract
        """
        k = '{0}_{1}_{2}'.format(sender, protocol, cost)
        self.thirdContract[k] = self.thirdContract[k]+1 if k in self.thirdContract else 1

    def addThirdDemand(self, sender, protocol, cost):
        """
        :param cType: contract type
        :param issuer: the issuer of the contract
        """
        k = '{0}_{1}_{2}'.format(sender, protocol, cost)
        self.receivedDemand[k] = self.receivedDemand[k] + 1 if k in self.receivedDemand else 1

    # def updateCost(self):
    def addIssueDemand(self, receiver, protocol, cost):
        k = '{0}_{1}_{2}'.format(receiver, protocol, cost)
        self.issuedDemand[k] = self.issuedDemand[k] + 1 if k in self.issuedDemand else 1


    def getthirdcontractsdemands(self):
        return (self.thirdContract, self.receivedDemand)

    def getCostRewards(self):
        woncontracts = {}
        # print self.receivedDemand
        for k, v in self.receivedDemand.items():
            g = re.search(r'.+_(\w+)_(\d+)', k).groups()
            protocol = g[0]
            cost = int(g[1])
            if protocol in woncontracts:
                for i in range(v):
                    woncontracts[protocol].append(cost)
            else:
                woncontracts[protocol] = []

        for k in woncontracts:
            woncontracts[k] = list2dict(woncontracts[k])
        #     print k
        #     print "The won count:", list2dict(woncontracts[k])
        #     print "Offer Count:", list2dict(self.costHistory[k])

        # costrewards = {}
        return woncontracts




