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
SpaceGroundLink class.
"""

from .transceiver import Transceiver

class SpaceGroundLink(Transceiver):
    def __init__(self, name=None, cost=0, size=1, capacity=1,
                 protocol=None, maxTransmitted=1, maxReceived=1):
        """
        @param name: the name of this space-to-ground link
        @type name: L{str}
        @param cost: the cost of this space-to-ground link
        @type cost: L{float}
        @param size: the size of this storage module
        @type size: L{float}
        @param capacity: the data capacity of this space-to-ground link
        @type capacity: L{int}
        @param protocol: the protocol of this space-to-ground link
        @type protocol: L{str}
        @param maxTransmitted: the max data transmitted by this space-to-ground link each turn
        @type maxTransmitted: L{int}
        @param maxReceived: the max data received by this space-to-ground link each turn
        @type maxReceived: L{int}
        """
        Transceiver.__init__(self, name=name, cost=cost,
                             size=size, capacity=capacity,
                             protocol=protocol,
                             maxTransmitted=maxTransmitted,
                             maxReceived=maxReceived)
    
    def couldTransmit(self, data, receiver, txLocation, rxLocation, context=None):
        """
        Checks if this space-to-ground link could transmit data (state-independent).
        @param data: the data to transmit
        @type data: L{Data}
        @param receiver: the receiver receiving the data
        @type receiver: L{Transceiver}
        @param txLocation: the transmitter location
        @type txLocation: L{Location}
        @param rxLocation: the receiver location
        @type rxLocation: L{Location}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        return super(SpaceGroundLink, self).couldTransmit(data, receiver) \
                and txLocation.isOrbit() \
                and rxLocation.isSurface() \
                and txLocation.sector == rxLocation.sector
        
    def couldReceive(self, data, transmitter, txLocation, rxLocation, context=None):
        """
        Checks if this space-to-ground link could receive data (state-independent).
        @param data: the data to transmit
        @type data: L{Data}
        @param transmitter: the transmitter transmitting the data
        @type transmitter: L{Transceiver}
        @param txLocation: the transmitter location
        @type txLocation: L{Location}
        @param rxLocation: the receiver location
        @type rxLocation: L{Location}
        @param context: the context
        @type context: L{Context}
        @return: L{bool}
        """
        return super(SpaceGroundLink, self).couldReceive(data, transmitter) \
                and txLocation.isOrbit() \
                and rxLocation.isSurface() \
                and txLocation.sector == rxLocation.sector
        
    def isSGL(self):
        """
        Checks if this is a space-to-ground link.
        @return: L{bool}
        """
        return True