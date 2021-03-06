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
Controller class.
"""

import logging

from .entity import Entity
from .contract import Contract

class Controller(Entity):
    def __init__(self, name=None):
        """
        @param name: the name of this controller
        @type name: L{str}
        """
        Entity.__init__(self, name=name)
        
    def getElements(self):
        """
        Gets the elements controlled by this controller.
        @return L{list}
        """
        return []
    
    def getFederates(self):
        """
        Gets the federates controlled by this controller.
        @return L{list}
        """
        return []
    
    def getContracts(self):
        """
        Gets the contracts controlled by this controller.
        @return L{list}
        """
        return []
    
    def liquidate(self, context):
        """
        Liquidates all assets.
        """
        for element in self.getElements():
            self.decommission(element)
        for contract in self.getContracts():
            self.resolve(contract, context)
    
    def canContract(self, demand, context):
        """
        Checks if this controller can contract a demand.
        @param demand: the demand to contract
        @type demand: L{Demand}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        return (demand in context.currentEvents
                and any(element.canSense(demand)
                        for element in self.getElements()))
    
    def canResolve(self, contract):
        """
        Checks if this controller can resolve a contract.
        @param contract: the contract to resolve
        @type contract: L{Contract}
        @return: L{bool}
        """
        return contract in self.getContracts()
    
    def canSense(self, demand, element, context):
        """
        Checks if this controller can sense a demand.
        @param demand: the demand to sense
        @type demand: L{Demand}
        @param element: the element to sense
        @type element: L{Element}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        return (element in self.getElements()
                and self.canContract(demand, context)
                or any(contract.demand is demand
                       for contract in self.getContracts())
                and element.canSense(demand))
    
    def senseAndStore(self, contract, element, context):
        """
        Senses and stores data for a contract.
        @param contract: the contract to sense and store
        @type contract: L{Contract}
        @param element: the element to sense
        @type element: L{Element}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        if contract not in self.getContracts():
            logging.warning('{0} does not control {1}.'
                        .format(self.name, contract.name))
        elif element not in self.getElements():
            logging.warning('{0} does not control {1}.'
                        .format(self.name, element.name))
        elif (self.canSense(contract.demand, element, context)
              and element.senseAndStore(contract)):
            for federate in [federate for federate in self.getFederates()
                             if federate.canSense(contract.demand, element, context)]:
                logging.info('{0} sensed and stored data for {1} using {2}'
                            .format(federate.name, contract.name, element.name))
                self.trigger('sense', federate, contract, element)
                return True
        else:
            logging.warning('{0} could not sense and store data for {1} using {2}'
                        .format(self.name, contract.name, element.name))
        return False
    
    def couldTransport(self, protocol, data, txElement,
                       rxElement, txLocation, rxLocation, context):
        """
        Checks if this controller could transport data between elements.
        @param protocol: the transmission protocol
        @type protocol: L{str}
        @param data: the data to transport
        @type data: L{Data}
        @param txElement: the transmitting element
        @type txElement: L{Element}
        @param rxElement: the receiving element
        @type rxElement: L{Element}
        @param txLocation: the transmitting location
        @type txLocation: L{Location}
        @param rxLocation: the receiving location
        @type rxLocation: L{Location}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        return (txElement.couldTransmit(protocol, data, rxElement,
                                        txLocation, rxLocation, context)
                and rxElement.couldReceive(protocol, data, txElement,
                                           txLocation, rxLocation, context)
                and ('o' in protocol
                     or next((federate for federate in self.getFederates()
                              if txElement in federate.elements), None)
                     is next((federate for federate in self.getFederates()
                              if rxElement in federate.elements), None)))
    
    def canTransport(self, protocol, data, txElement, rxElement, context):
        """
        Checks if this controller can transport data between elements.
        @param protocol: the transmission protocol
        @type protocol: L{str}
        @param data: the data to transport
        @type data: L{Data}
        @param txElement: the transmitting element
        @type txElement: L{Element}
        @param rxElement: the receiving element
        @type rxElement: L{Element}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        return (self.couldTransport(protocol, data, txElement, rxElement,
                                    txElement.location, rxElement.location, context)
                and txElement in self.getElements()
                and rxElement in self.getElements()
                and txElement.canTransmit(protocol, data, rxElement, context)
                and rxElement.canReceive(protocol, data, txElement, context))
    
    def transport(self, protocol, data, txElement, rxElement, context):
        """
        Transports data between elements.
        @param protocol: the transmission protocol
        @type protocol: L{str}
        @param data: the data to transport
        @type data: L{Data}
        @param txElement: the transmitting element
        @type txElement: L{Element}
        @param rxElement: the receiving element
        @type rxElement: L{Element}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        if txElement not in self.getElements():
            logging.warning('{0} does not control {1}.'
                        .format(self.name, txElement.name))
        elif rxElement not in self.getElements():
            logging.warning('{0} does not control {1}.'
                        .format(self.name, rxElement.name))
        elif self.canTransport(protocol, data, txElement, rxElement, context):
            if not txElement.transmit(protocol, data, rxElement, context):
                logging.warning('{0} could not transmit {1} to {2} with {3}'
                             .format(txElement.name, data,
                                     rxElement.name, protocol))
            elif not rxElement.receive(protocol, data, txElement, context):
                logging.warning('{0} could not receive {1} from {2} with {3}'
                             .format(rxElement.name, str(data),
                                     txElement.name, protocol))
            else:
                logging.info('{0} transported {1} from {2} to {3} with {4}'
                            .format(self.name, str(data), txElement.name,
                                    rxElement.name, protocol))
                self.trigger('transport', self, protocol, data, txElement, rxElement)
                return True
        else:
            logging.warning('{0} could not transport {1} between {2} and {3} with {4}'
                         .format(self.name, str(data), txElement.name,
                                 rxElement.name, protocol))
        return False
    
    def deleteData(self, contract):
        """
        Deletes data for a contract.
        @param contract: the contract
        @type contract: L{Contract}
        """
        for element in self.getElements():
            for module in element.modules:
                for d in module.data[:]:
                    if (d.contract is contract
                        and module.canTransferOut(d)):
                        module.transferOut(d)
        
    def contract(self, demand, context):
        """
        Contracts a demand.
        @param demand: the demand to contract
        @type demand: L{Demand}
        @param context: the context
        @type context: L{Context}
        @return: L{Contract}
        """
        if self.canContract(demand, context):
            for federate in [federate for federate in self.getFederates()
                             if federate.canContract(demand, context)]:
                context.currentEvents.remove(demand)
                contract = Contract(demand)
                federate.contracts.append(contract)
                logging.info('{0} contracted for {1}'
                            .format(federate.name, demand.name))
                self.trigger('contract', federate, demand)
                return contract
        logging.warning('{0} could not contract for {1}'
                    .format(self.name, demand.name))
        return None
    
    def resolve(self, contract, context):
        """
        Resolves a contract.
        @param contract: the transmission protocol
        @type contract: L{Contract}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        if self.canResolve(contract):
            for federate in [federate for federate in self.getFederates()
                             if federate.canResolve(contract)]:
                element = context.getDataElement(contract)
                value = (contract.getValue()
                         if contract.isCompleted(element.location
                                                 if element is not None
                                                 else None)
                         else contract.demand.getDefaultValue())
                federate.cash += value
                federate.contracts.remove(contract)
                context.pastEvents.append(contract.demand)
                self.deleteData(contract)
                logging.info('{0} resolved {1} for {2} cash'
                            .format(federate.name, contract.name, value))
                self.trigger('resolve', federate, contract, value)
                return True
        logging.warning('{0} could not resolve {1}.'
                    .format(self.name, contract.name))
        return False
    
    def canExchange(self, amount, debtor, creditor):
        """
        Checks if this federation can exchange cash between federates.
        @param amount: the amount of cash to exchange
        @type amount: L{float}
        @param debtor: the debtor federate
        @type debtor: L{Federate}
        @param creditor: the creditor federate
        @type creditor: L{Federate}
        @return: L{bool}
        """
        return (debtor in self.getFederates()
                and creditor in self.getFederates()
                and amount > debtor.cash)
    
    def exchange(self, amount, debtor, creditor):
        """
        Exchanges cash between two federates in this federation.
        @param amount: the amount of cash to exchange
        @type amount: L{float}
        @param debtor: the debtor federate
        @type debtor: L{Federate}
        @param creditor: the creditor federate
        @type creditor: L{Federate}
        @return: L{bool}
        """
        if debtor not in self.getFederates():
            logging.warning('{} does not control {}'
                            .format(self.name, debtor.name))
        elif creditor not in self.getFederates():
            logging.warning('{} does not control {}'
                            .format(self.name, creditor.name))
        elif amount > debtor.cash:
            logging.warning('{} exceeds {} cash.'
                            .format(amount, debtor.name))
        elif self.canExchange(amount, debtor, creditor):
            debtor.cash -= amount
            creditor.cash += amount
            logging.info('{} paid {} to {}'
                         .format(debtor.name, amount, creditor.name))
            self.trigger('exchange', self, amount, debtor, creditor)
            return True
        return False