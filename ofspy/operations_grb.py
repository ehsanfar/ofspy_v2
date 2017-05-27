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
Operations implementation for gurobi module.
"""

import random
import logging

from .operations import Operations

from gurobipy import Model, LinExpr, GRB, GurobiError

class DynamicOperations(Operations):
    def __init__(self, planningHorizon=6, storagePenalty=-100, islPenalty=-10):
        """
        @param planningHorizon: the planning horizon
        @type planningHorizon: L{int}
        @param storagePenalty: the storage opportuntiy cost
        @type storagePenalty: L{float}
        @param islPenalty: the ISL opportuntiy cost
        @type islPenalty: L{float}
        """
        # print "This is DynamicOperations"
        super(DynamicOperations, self).__init__()
        self.planningHorizon = planningHorizon
        self.storagePenalty = storagePenalty
        self.islPenalty = islPenalty
    
    def execute(self, controller, context):
        """
        Executes this operations model.
        @param controller: the controller for this operations model
        @type controller: L{Entity}
        @param context: the context of operations
        @type context: L{Context}
        """
        minTime = context.time
        maxTime = (context.time + self.planningHorizon
                   if context.maxTime is None else
                   min(context.maxTime, context.time + self.planningHorizon))

        try:
            lp = Model('OFS LP for {}'.format(controller.name))

            S = []      # S[i][j]: satellite i senses demand j
            E_d = []    # E_d[t][i][j]: at time t satellite i holds data for demand j
            E_c0 = []   # E_c0[i][j]: satellite i initially holds data for contract j
            E_c = []    # E_c[t][i][j]: at time t satellite i holds data for contract j
            T_d = []    # T_d[t][i][j][k][l]: at time t transmit data from satellite i to ground station j using protocol k for demand l
            T_c = []    # T_c[t][i][j][k][l]: at time t transmit data from satellite i to ground station j using protocol k for contract l
            L_d = []    # L_d[t][i][j][k][l]: at time t transmit data from isl satellite i to isl satellite j using protocol k for demand l
            L_c = []    # L_c[t][i][j][k][l]: at time t transmit data from isl satellite i to isl satellite j using protocol k for contract l
            R_d = []    # R_d[t][i][j]: at time t resolve data in system i for demand j
            R_c = []    # R_c[t][i][j]: at time t resolve data in system i for contract j
            J = LinExpr()   # objective function
            
            demands = [e for e in context.currentEvents if e.isDemand()]
            elements = controller.getElements()
            federates = controller.getFederates()
            satellites = [e for e in elements if e.isSpace()]
            satellitesISL = [e for e in satellites 
                if any(m.isTransceiver() and m.isISL() for m in e.modules)]
            stations = [e for e in elements if e.isGround()]
            contracts = controller.getContracts()
            print "All contracts (centralized):", [a.name for a in contracts]
            print "All demands (centralized):", [a.name for a in demands]

            protocolsSGL = list(set([m.protocol for e in elements 
                for m in e.modules if m.isTransceiver() and m.isSGL()]))
            protocolsISL = list(set([m.protocol for e in elements 
                for m in e.modules if m.isTransceiver() and m.isISL()]))
            phenomena = ['VIS','SAR',None]

            # ''' ********************TEST TEST TEST************************ '''
            # # print "Length of federates:", len(federates)
            # federate1, federate2 = [federate for federation in context.federations for federate in federation.federates]
            #
            # ownElements1 = [e for e in elements if e in federate1.elements]
            # ownElements2 = [e for e in elements if e in federate2.elements]
            #
            # ownSatellites1 = [e for e in ownElements1 if e.isSpace()]
            # ownSatellites2 = [e for e in ownElements2 if e.isSpace()]
            # ownStations1 = [e for e in ownElements1 if e.isGround()]
            # ownStations2 = [e for e in ownElements2 if e.isGround()]
            #
            # ''' ********************FINISH************************ '''

            for i, satellite in enumerate(satellites):
                S.insert(i, [])
                for j, demand in enumerate(demands):
                    # satellite i senses data for demand j
                    S[i].insert(j, lp.addVar(vtype=GRB.BINARY, 
                        name='{}-S-{}'.format(satellite.name, demand.name)))
                    # constrain sensing per satellite
                    lp.addConstr(S[i][j] <= (1 if satellite.canSense(demand) else 0),
                                 '{} can sense {}'.format(satellite.name, demand.name))
                for phenomenon in phenomena:
                    r = LinExpr()
                    for j, demand in enumerate(demands):
                        if phenomenon is None or demand.phenomenon == phenomenon:
                            r.add(S[i][j], demand.size)
                    # constrain maximum data sensed by satellite
                    lp.addConstr(r <= min(
                        satellite.getMaxSensed(phenomenon) - satellite.getSensed(phenomenon),
                        satellite.getMaxStored(phenomenon) - satellite.getStored(phenomenon)),
                        '{} max sense {}'.format(satellite.name, phenomenon))
                # set initial data stored
                E_c0.insert(i, [])
                for j, contract in enumerate(contracts):
                    E_c0[i].insert(j, 1 if any(d.contract is contract 
                        for m in satellite.modules for d in m.data) else 0)
            
            for j, demand in enumerate(demands):
                r = LinExpr()
                for i, satellite in enumerate(satellites):
                    r.add(S[i][j], 1)
                lp.addConstr(r <= 1, '{} max sensed'.format(demand.name))
            
            for t, time in enumerate(range(minTime, maxTime+1)):
                E_d.insert(t, [])
                E_c.insert(t, [])
                for i, satellite in enumerate(satellites):
                    E_d[t].insert(i, [])
                    E_c[t].insert(i, [])
                    for j, demand in enumerate(demands):
                        stoPen = self.getStoragePenalty(demand.phenomenon, satellite, context, minTime, t, type='independent')
                        # satellite i stores data for new contract j
                        E_d[t][i].insert(j, lp.addVar(vtype=GRB.BINARY, 
                            name='{}-E-{}@{}'.format(satellite.name, demand.name, time)))
                        # penalty for opportunity cost
                        J.add(E_d[t][i][j], demand.size*stoPen)
                    for j, contract in enumerate(contracts):
                        stoPen = self.getStoragePenalty(contract.demand.phenomenon, satellite, context, minTime, t, type='independent')
                        # satellite i stores data for contract j
                        E_c[t][i].insert(j, lp.addVar(vtype=GRB.BINARY,
                            name='{}-E-{}@{}'.format(satellite.name, contract.name, time)))
                        # penalty for opportunity cost
                        J.add(E_c[t][i][j], contract.demand.size*stoPen)
                    for phenomenon in phenomena:
                        r = LinExpr()
                        for j, demand in enumerate(demands):
                            if phenomenon is None or demand.phenomenon == phenomenon:
                                r.add(E_d[t][i][j], demand.size)
                        for j, contract in enumerate(contracts):
                            if phenomenon is None or contract.demand.phenomenon == phenomenon:
                                r.add(E_c[t][i][j], contract.demand.size)
                        # constrain data stored in satellite
                        lp.addConstr(r <= satellite.getMaxStored(phenomenon), 
                                     '{} max store {} at {}'.format(
                                     satellite.name, phenomenon, time))
                T_d.insert(t, [])
                T_c.insert(t, [])
                for i, satellite in enumerate(satellites):
                    T_d[t].insert(i, [])
                    T_c[t].insert(i, [])
                    txLocation = context.propagate(satellite.location, time-context.time)
                    for j, station in enumerate(stations):
                        T_d[t][i].insert(j, [])
                        T_c[t][i].insert(j, [])
                        rxLocation = context.propagate(station.location, time-context.time)
                        for k, protocol in enumerate(protocolsSGL):
                            T_d[t][i][j].insert(k, [])
                            T_c[t][i][j].insert(k, [])
                            r = LinExpr()
                            maxSize = 0
                            for l, demand in enumerate(demands):
                                T_d[t][i][j][k].insert(l, lp.addVar(
                                    vtype=GRB.BINARY,
                                    name='{}-T({}/{})-{}@{}'.format(
                                    satellite.name, demand.name, protocol, 
                                    station.name, time)))
                                r.add(T_d[t][i][j][k][l], demand.size)
                                maxSize = max(maxSize, demand.size if controller.couldTransport(
                                    protocol, demand.generateData(), satellite, 
                                    station, txLocation, rxLocation, context) 
                                    and not demand.isDefaultedAt(time-context.time) else 0)
                            for l, contract in enumerate(contracts):
                                T_c[t][i][j][k].insert(l, lp.addVar(
                                    vtype=GRB.BINARY, 
                                    name='{}-T({}/{})-{}@{}'.format(
                                        satellite.name, contract.name,
                                        protocol, station.name, time)))
                                r.add(T_c[t][i][j][k][l], contract.demand.size)
                                maxSize = max(maxSize, contract.demand.size if controller.couldTransport(
                                    protocol, contract.demand.generateData(), satellite,  
                                    station, txLocation, rxLocation, context)
                                    and not contract.demand.isDefaultedAt(
                                    contract.elapsedTime+time-context.time) else 0)
                            # constrain transmission by visibility
                            lp.addConstr(r <= maxSize, '{}-{} visibility {} at {}'.format(
                                         satellite.name, station.name, protocol, time))
                for i, satellite in enumerate(satellites):
                    for k, protocol in enumerate(protocolsSGL):
                        r = LinExpr()
                        for j, station in enumerate(stations):
                            for l, demand in enumerate(demands):
                                r.add(T_d[t][i][j][k][l], demand.size)
                            for l, contract in enumerate(contracts):
                                r.add(T_c[t][i][j][k][l], contract.demand.size)
                        # constrain data transmitted by satellite
                        lp.addConstr(r <= (satellite.getMaxTransmitted(protocol)
                            - (satellite.getTransmitted(protocol) if time == minTime else 0)),
                            '{} max transmit {} at {}'.format(satellite.name, protocol, time))
                for j, station in enumerate(stations):
                    for k, protocol in enumerate(protocolsSGL):
                        r = LinExpr()
                        for i, satellite in enumerate(satellites):
                            for l, demand in enumerate(demands):
                                r.add(T_d[t][i][j][k][l], demand.size)
                            for l, contract in enumerate(contracts):
                                r.add(T_c[t][i][j][k][l], contract.demand.size)
                        # constrain data received by station
                        lp.addConstr(r <= station.getMaxReceived(protocol) 
                            - (station.getReceived(protocol) if time == minTime else 0),
                            '{} max receive {} at {}'.format(station.name, protocol, time))
                L_d.insert(t, [])
                L_c.insert(t, [])
                for i, txSatellite in enumerate(satellitesISL):
                    L_d[t].insert(i, [])
                    L_c[t].insert(i, [])
                    txLocation = context.propagate(txSatellite.location, time-context.time)
                    for j, rxSatellite in enumerate(satellitesISL):
                        L_d[t][i].insert(j, [])
                        L_c[t][i].insert(j, [])
                        rxLocation = context.propagate(rxSatellite.location, time-context.time)
                        for k, protocol in enumerate(protocolsISL):
                            L_d[t][i][j].insert(k, [])
                            L_c[t][i][j].insert(k, [])
                            r = LinExpr()
                            maxSize = 0
                            for l, demand in enumerate(demands):
                                L_d[t][i][j][k].insert(l, lp.addVar(
                                    vtype=GRB.BINARY,
                                    name='{}-T({}/{})-{}@{}'.format(
                                    txSatellite.name, demand.name,
                                    protocol, rxSatellite.name, time)))
                                # small penalty for opportunity cost
                                J.add(L_d[t][i][j][k][l], self.islPenalty*demand.size)
                                r.add(L_d[t][i][j][k][l], demand.size)
                                maxSize = max(maxSize, demand.size if controller.couldTransport(
                                    protocol, demand.generateData(), txSatellite, 
                                    rxSatellite, txLocation, rxLocation, context)
                                    and not demand.isDefaultedAt(time-context.time) else 0)
                            for l, contract in enumerate(contracts):
                                L_c[t][i][j][k].insert(l, lp.addVar(
                                    vtype=GRB.BINARY,
                                    name='{}-T({}/{})-{}@{}'.format(
                                        txSatellite.name, contract.name,
                                        protocol, rxSatellite.name, time)))
                                # small penalty for opportunity cost
                                J.add(L_c[t][i][j][k][l], self.islPenalty*contract.demand.size)
                                r.add(L_c[t][i][j][k][l], contract.demand.size)
                                maxSize = max(maxSize, contract.demand.size if controller.couldTransport(
                                    protocol, contract.demand.generateData(), 
                                    txSatellite, rxSatellite, txLocation, rxLocation, context)
                                    and not contract.demand.isDefaultedAt(
                                    contract.elapsedTime+time-context.time) else 0)
                            # constrain transmission by visibility
                            lp.addConstr(r <= maxSize, '{}-{} visibility {} at {}'.format(
                                         txSatellite.name, rxSatellite.name, protocol, time))
                for i, txSatellite in enumerate(satellitesISL):
                    for k, protocol in enumerate(protocolsISL):
                        r = LinExpr()
                        for j, rxSatellite in enumerate(satellitesISL):
                            for l, demand in enumerate(demands):
                                r.add(L_d[t][i][j][k][l], demand.size)
                            for l, contract in enumerate(contracts):
                                r.add(L_c[t][i][j][k][l], contract.demand.size)
                        # constrain data transmitted by satellite
                        lp.addConstr(r <= (txSatellite.getMaxTransmitted(protocol) 
                            - (txSatellite.getTransmitted(protocol) if time == minTime else 0)),
                            '{} max transmit {} at {}'.format(txSatellite.name, protocol, time))
                for j, rxSatellite in enumerate(satellitesISL):
                    for k, protocol in enumerate(protocolsISL):
                        r = LinExpr()
                        for i, txSatellite in enumerate(satellitesISL):
                            for l, demand in enumerate(demands):
                                r.add(L_d[t][i][j][k][l], demand.size)
                            for l, contract in enumerate(contracts):
                                r.add(L_c[t][i][j][k][l], contract.demand.size)
                        # constrain data received by station
                        lp.addConstr(r <= (rxSatellite.getMaxReceived(protocol) 
                            - (rxSatellite.getReceived(protocol) if time == minTime else 0)),
                            '{} max receive {} at {}'.format(rxSatellite.name, protocol, time))
                R_d.insert(t, [])
                R_c.insert(t, [])
                for i, element in enumerate(elements):
                    location = context.propagate(element.location, time-context.time)
                    R_d[t].insert(i, [])
                    R_c[t].insert(i, [])
                    for j, demand in enumerate(demands):
                        R_d[t][i].insert(j, lp.addVar(vtype=GRB.BINARY,
                            name='{}-R-{}@{}'.format(element.name, demand.name, time)))
                        J.add(R_d[t][i][j], demand.getValueAt(time-context.time)
                              if demand.isCompletedAt(location)
                              else demand.getDefaultValue())
                    for j, contract in enumerate(contracts):
                        R_c[t][i].insert(j, lp.addVar(vtype=GRB.BINARY,
                            name='{}-R-{}@{}'.format(element.name, contract.name, time)))
                        J.add(R_c[t][i][j], contract.demand.getValueAt(
                            contract.elapsedTime + time-context.time)
                              if contract.demand.isCompletedAt(location)
                              else contract.demand.getDefaultValue())
                for i, satellite in enumerate(satellites):
                    R_i = elements.index(satellite)
                    for j, demand in enumerate(demands):
                        r = LinExpr()
                        if time==minTime:
                            r.add(S[i][j], 1)
                        else:
                            r.add(E_d[t-1][i][j],1)
                        r.add(E_d[t][i][j],-1)
                        r.add(R_d[t][R_i][j],-1)
                        for k, station in enumerate(stations):
                            for l, protocol in enumerate(protocolsSGL):
                                r.add(T_d[t][i][k][l][j],-1)
                        if satellite in satellitesISL:
                            isl_i = satellitesISL.index(satellite)
                            for k, rxSatellite in enumerate(satellitesISL):
                                for l, protocol in enumerate(protocolsISL):
                                    r.add(L_d[t][isl_i][k][l][j],-1)
                                    r.add(L_d[t][k][isl_i][l][j],1)
                        # constrain net flow of new contracts at each satellite
                        lp.addConstr(r == 0, '{} net flow {} at {}'.format(
                            satellite.name, demand.name, time))
                    for j, contract in enumerate(contracts):
                        r = LinExpr()
                        if time==minTime:
                            # existing contracts are initial conditions
                            pass
                        else:
                            r.add(E_c[t-1][i][j],1)
                        r.add(E_c[t][i][j],-1)
                        r.add(R_c[t][R_i][j],-1)
                        for k, station in enumerate(stations):
                            for l, protocol in enumerate(protocolsSGL):
                                r.add(T_c[t][i][k][l][j],-1)
                        if satellite in satellitesISL:
                            isl_i = satellitesISL.index(satellite)
                            for k, rxSatellite in enumerate(satellitesISL):
                                for l, protocol in enumerate(protocolsISL):
                                    r.add(L_c[t][isl_i][k][l][j],-1)
                                    r.add(L_c[t][k][isl_i][l][j],1)
                        # constrain net flow of contracts at each satellite
                        lp.addConstr(r == (-1*(E_c0[i][j] if time == minTime else 0)),
                                     '{} net flow {} at {}'.format(
                                     satellite.name, contract.name, time))
                if time+1 > maxTime and self.planningHorizon > 0:
                    for i, satellite in enumerate(satellites):
                        r = LinExpr()
                        for j, demand in enumerate(demands):
                            r.add(E_d[t][i][j],1)
                        for j, contract in enumerate(contracts):
                            r.add(E_c[t][i][j],1)
                        # constrain boundary flow of each satellite
                        lp.addConstr(r == 0, '{} boundary flow'.format(satellite.name))
                for k, station in enumerate(stations):
                    R_k = elements.index(station)
                    for j, demand in enumerate(demands):
                        r = LinExpr()
                        r.add(R_d[t][R_k][j],-1)
                        for i, satellite in enumerate(satellites):
                            for l, protocol in enumerate(protocolsSGL):
                                r.add(T_d[t][i][k][l][j],1)
                        # constrain net flow of new contracts at each station
                        lp.addConstr(r == 0, '{} net flow {} at {}'.format(
                            station.name, demand.name, time))
                    for j, contract in enumerate(contracts):
                        r = LinExpr()
                        r.add(R_c[t][R_k][j],-1)
                        for i, satellite in enumerate(satellites):
                            for l, protocol in enumerate(protocolsSGL):
                                r.add(T_c[t][i][k][l][j],1)
                        # constrain net flow of contracts at each station
                        lp.addConstr(r == 0, '{} net flow {} at {}'.format(
                            station.name, contract.name, time))
            for federate in federates:
                r = LinExpr()
                for j, demand in enumerate(demands):
                    if federate.canContract(demand, context): # TODO does not consider priority
                        for i, element in enumerate(elements):
                            location = context.propagate(element.location, time-context.time)
                            r.add(R_d[0][i][j], demand.getValueAt(0) 
                                if demand.isCompletedAt(location)
                                else demand.getDefaultValue())
                for j, contract in enumerate(contracts):
                    if contract in federate.contracts:
                        for i, element in enumerate(elements):
                            location = context.propagate(element.location, time-context.time)
                            r.add(R_c[0][i][j], contract.demand.getValueAt(contract.elapsedTime)
                                if contract.demand.isCompletedAt(location)
                                else contract.demand.getDefaultValue())
                lp.addConstr(r >= -1 - federate.cash, 
                             '{} net cash must be positive'.format(federate.name))
            
            lp.setObjective(J, GRB.MAXIMIZE)
            lp.setParam('OutputFlag', False)
            lp.optimize()

            # ''' ********************TEST TEST TEST************************ '''
            #
            # Td_1 = Td_0 = Tc_1 = Tc_0 = Ld_1 = Ld_0 = Lc_1 = Lc_0 = 0
            #
            # for t, time in enumerate(range(minTime, maxTime + 1)):
            #     for i, satellite in enumerate(satellites):
            #         for j, station in enumerate(stations):
            #             for k, protocol in enumerate(protocolsSGL):
            #                 for l, demand in enumerate(demands):
            #                     # print T_d[t][i][j][k][l].x
            #                     if T_d[t][i][j][k][l].x > 0.5:
            #                         print "Own SGL: ", (satellite in ownSatellites1), (station in ownStations1), (satellite in ownSatellites2), (station in ownStations2)
            #                         Td_1 += 1
            #                     else:
            #                         Td_0 += 1
            #
            #                 for l, contract in enumerate(contracts):
            #                     # print T_c[t][i][j][k][l].x
            #                     if T_c[t][i][j][k][l].x > 0:
            #                         Tc_1 += 1
            #                     else:
            #                         Tc_0 += 1
            #     for i, txSatellite in enumerate(satellitesISL):
            #         for j, rxSatellite in enumerate(satellitesISL):
            #             for k, protocol in enumerate(protocolsISL):
            #                 for l, demand in enumerate(demands):
            #                     if L_d[t][i][j][k][l].x > 0:
            #                         print "Own ISL: ", (txSatellite in ownSatellites1), (rxSatellite in ownStations1), (txSatellite in ownSatellites2), (rxSatellite in ownStations2)
            #                         Ld_1 += 1
            #                     else:
            #                         Ld_0 += 1
            #
            #                 for l, contract in enumerate(contracts):
            #                     if L_c[t][i][j][k][l].x > 0:
            #                         Lc_1 += 1
            #                     else:
            #                         Lc_0 += 1
            #
            # print Td_0, Td_1, Tc_0, Tc_1, Ld_0, Ld_1, Lc_0, Lc_1
            # ''' ********************FINISH************************ '''
            
            def _transportContract(operations, satellite, contract, context):
                i = satellites.index(satellite)
                R_i = elements.index(satellite)
                j = contracts.index(contract)
                data = context.getData(contract)
                if data is not None:
                    if R_c[0][R_i][j].x > 0:
                        controller.resolve(contract, context)
                    elif E_c[0][i][j].x > 0:
                        satellite.store(data)
                    elif any(any(T_c[0][i][k][l][j].x
                                 for k, station in enumerate(stations))
                             for l, protocol in enumerate(protocolsSGL)):
                        for k, station in enumerate(stations):
                            for l, protocol in enumerate(protocolsSGL):
                                if(T_c[0][i][k][l][j].x):
                                    controller.transport(protocol, data, satellite, 
                                                         station, context)
                                    # print "transport contract:", [f.cash for f in federates],
                                    controller.resolve(contract, context)
                                    # print [f.cash for f in federates]
                    elif satellite in satellitesISL:
                        isl_i = satellitesISL.index(satellite)
                        for k, rxSatellite in enumerate(satellitesISL):
                            for l, protocol in enumerate(protocolsISL):
                                if(L_c[0][isl_i][k][l][j].x):
                                    controller.transport(protocol, data, satellite, 
                                                         rxSatellite, context)
                                    _transportContract(operations, rxSatellite, 
                                                       contract, context)
            def _transportDemand(operations, satellite, demand, context):
                i = satellites.index(satellite)
                R_i = elements.index(satellite)
                j = demands.index(demand)
                contract = context.getContract(demand)
                data = context.getData(contract)
                if contract is not None and data is not None:
                    if R_d[0][R_i][j].x > 0:
                        controller.resolve(contract, context)
                    elif E_d[0][i][j].x > 0:
                        satellite.store(data)
                    elif any(any(T_d[0][i][k][l][j].x
                                 for k, station in enumerate(stations))
                             for l, protocol in enumerate(protocolsSGL)):
                        for k, station in enumerate(stations):
                            for l, protocol in enumerate(protocolsSGL):
                                if(T_d[0][i][k][l][j].x):
                                    controller.transport(protocol, data, satellite, 
                                                         station, context)
                                    # print "transport demand:", [f.cash for f in federates],
                                    controller.resolve(contract, context)
                                    # print [f.cash for f in federates]
                    elif satellite in satellitesISL:
                        isl_i = satellitesISL.index(satellite)
                        for k, rxSatellite in enumerate(satellitesISL):
                            for l, protocol in enumerate(protocolsISL):
                                if(L_d[0][isl_i][k][l][j].x):
                                    controller.transport(protocol, data, satellite, 
                                                         rxSatellite, context)
                                    _transportDemand(operations, rxSatellite, 
                                                     demand, context)
            
            # first, transport contracts to resolution
            # print "Before Cash:", federate1.cash, federate2.cash
            for j, contract in enumerate(contracts):
                if any(R_c[0][i][j].x > 0 for i, element in enumerate(elements)):
                    logging.debug('Transporting contract {} for resolution...'
                                  .format(contract.name))
                    satellite = context.getDataElement(contract)
                    # print "contract resolution"
                    _transportContract(self, satellite, contract, context)
                
            # second, sense and transport demands to resolution
            # print [f.cash for f in federates],
            for j, demand in enumerate(demands):
                if any(R_d[0][i][j].x > 0 for i, element in enumerate(elements)):
                    logging.debug('Sensing and transporting demand {} for resolution...'
                                  .format(demand.name))
                    satellite = next(e for i, e in enumerate(satellites) if S[i][j].x > 0)
                    contract = controller.contract(demand, context)
                    controller.senseAndStore(contract, satellite, context)
                    # print "demand resolution"
                    _transportDemand(self, satellite, demand, context)
            
            # third, sense all demands to be stored
            for j, demand in enumerate(demands):
                if (all(R_d[0][i][j].x < 1 for i, element in enumerate(elements))
                    and any(S[i][j].x > 0 for i, element in enumerate(satellites))):
                    logging.debug('Sensing demand {} for storage...'.format(demand.name))
                    satellite = next(e for i, e in enumerate(satellites) if S[i][j].x > 0)
                    contract = controller.contract(demand, context)
                    controller.senseAndStore(contract, satellite, context)
            
            # fourth, transport demands to storage
            # print [f.cash for f in federates],
            for j, demand in enumerate(demands):
                if (all(R_d[0][i][j].x < 1 for i, element in enumerate(elements))
                    and any(S[i][j].x > 0 for i, element in enumerate(satellites))):
                    logging.debug('Transporting demand {} for storage...'.format(demand.name))
                    satellite = next(e for i, e in enumerate(satellites) if S[i][j].x > 0)
                    # print "demand storage"
                    _transportDemand(self, satellite, demand, context)
            
            # finally, transport contracts to storage
            # print [f.cash for f in federates],
            for j, contract in enumerate(contracts):
                if all(R_c[0][i][j].x < 1 for i, element in enumerate(elements)):
                    logging.debug('Transporting contract {} for storage...'
                                  .format(contract.name))
                    satellite = context.getDataElement(contract)
                    # print "contract storage"
                    _transportContract(self, satellite, contract, context)
            # print "After Cash:", federate1.cash, federate2.cash
        except GurobiError as e:
            print('Error code ' + str(e.errno) + ": " + str(e))
        except AttributeError:
            print('Encountered an attribute error')



class VarCostDynamicOperations(DynamicOperations):
    def __init__(self, planningHorizon=6, storagePenalty=-100,
                 islPenalty=-10, costSGL=50, costISL=20):
        """
        @param planningHorizon: the planning horizon
        @type planningHorizon: L{int}
        @param storagePenalty: the storage opportuntiy cost
        @type storagePenalty: L{float}
        @param islPenalty: the ISL opportuntiy cost
        @type islPenalty: L{float}
        @param costSGL: the cost to use SGL
        @type costSGL: L{float}
        @param costISL: the cost to use ISL
        @type costISL: L{float}
        """

        super(VarCostDynamicOperations, self).__init__(
            planningHorizon, storagePenalty, islPenalty)
        self.costSGL = costSGL
        self.costISL = costISL
        # print "This is FixedCostDynamicOperations", self.costISL, self.costSGL

    def execute(self, controller, context):
        """
        Executes this operations model.
        @param controller: the controller for this operations model
        @type controller: L{Entity}
        @param context: the context of operations
        @type context: L{Context}
        """
        minTime = context.time
        maxTime = (context.time + self.planningHorizon
                   if context.maxTime is None else
                   min(context.maxTime, context.time + self.planningHorizon))
        allElements = controller.getElements()


        allSatellites = [e for e in allElements if e.isSpace()]
        allSatellitesISL = [e for e in allSatellites
                            if any(m.isTransceiver() and m.isISL()
                                   for m in e.modules)]
        allStations = [e for e in allElements if e.isGround()]

        protocolsSGL = list(set([m.protocol for e in allElements
                                 for m in e.modules
                                 if m.isTransceiver()
                                 and m.isSGL()]))
        protocolsISL = list(set([m.protocol for e in allElements
                                 for m in e.modules
                                 if m.isTransceiver()
                                 and m.isISL()]))
        phenomena = ['VIS','SAR',None]
        for a in allElements:
            if 'Sat' in a.name:
                # print "Min and max time:", minTime, maxTime
                stoPen = self.getStoragePenalty('VIS', a, context, minTime, 0)
                stoPen = self.getStoragePenalty('SAR', a, context, minTime, 0)
                # print "Operatins_grb, storage penalty:", stoPen
                a.trigger('storage', a)

        federates = controller.getFederates()
        random.shuffle(federates, context.orderStream.random)
        for f in federates:
            f.setCost('oISL', self.costISL)
            f.setCost('oSGL', self.costSGL)

        for federate in federates:
            try:
                lp = Model('OFS LP for {}'.format(controller.name))

                S = []      # S[i][j]: own satellite i senses demand j
                E_d = []    # E_d[t][i][j]: at time t own satellite i holds data for demand j
                E_c0 = []   # E_c0[i][j]: own satellite i initially holds data for own contract j
                E_c = []    # E_c[t][i][j]: at time t own satellite i holds data for own contract j
                T_d = []    # T_d[t][i][j][k][l]: at time t transmit data from satellite i to ground station j using protocol k for demand l
                T_c = []    # T_c[t][i][j][k][l]: at time t transmit data from satellite i to ground station j using protocol k for contract l
                L_d = []    # L_d[t][i][j][k][l]: at time t transmit data from isl satellite i to isl satellite j using protocol k for demand l
                L_c = []    # L_c[t][i][j][k][l]: at time t transmit data from isl satellite i to isl satellite j using protocol k for contract l
                R_d = []    # R_d[t][i][j]: at time t resolve data in system i for demand j
                R_c = []    # R_c[t][i][j]: at time t resolve data in system i for contract j
                J = LinExpr()   # objective function
                cost_dict = {} # the cost of SGL and ISL supply from one federate to other

                demands = [e for e in context.currentEvents if e.isDemand()]
                ownElements = [e for e in controller.getElements()
                    if e in federate.elements]
                ownSatellites = [e for e in ownElements if e.isSpace()]
                ownStations = [e for e in ownElements if e.isGround()]
                ownContracts = [c for c in controller.getContracts()
                    if c in federate.contracts]

                # print "demands (federated):", [a.name for a in demands]
                # ownContracts = []
                # print "own contracts (federated):", [a.name for a in ownContracts]

                for i, satellite in enumerate(ownSatellites):
                    S.insert(i, [])
                    for j, demand in enumerate(demands):
                        # satellite i senses data for demand j
                        S[i].insert(j, lp.addVar(vtype=GRB.BINARY,
                            name='{}-S-{}'.format(satellite.name, demand.name)))
                        # constrain sensing per satellite
                        canSense = satellite.canSense(demand)
                        satellite.addDemand(demand, canSense)
                        # print "All demand counter: ", satellite.sensedDemandCounter, satellite.allDemandCounter

                        lp.addConstr(S[i][j] <= (1 if canSense else 0),
                                     '{} can sense {}'.format(satellite.name, demand.name))
                    for phenomenon in phenomena:
                        r = LinExpr()
                        for j, demand in enumerate(demands):
                            if phenomenon is None or demand.phenomenon == phenomenon:
                                r.add(S[i][j], demand.size)
                        # constrain maximum data sensed by satellite
                        lp.addConstr(r <= min(
                            satellite.getMaxSensed(phenomenon) - satellite.getSensed(phenomenon),
                            satellite.getMaxStored(phenomenon) - satellite.getStored(phenomenon)),
                            '{} max sense {}'.format(satellite.name, phenomenon))
                    # set initial data stored
                    E_c0.insert(i, [])
                    for j, contract in enumerate(ownContracts):
                        E_c0[i].insert(j, 1 if any(d.contract is contract
                            for m in satellite.modules for d in m.data) else 0)

                for j, demand in enumerate(demands):
                    r = LinExpr()
                    for i, satellite in enumerate(ownSatellites):
                        r.add(S[i][j], 1)
                    lp.addConstr(r <= 1, '{} max sensed'.format(demand.name))

                for t, time in enumerate(range(minTime, maxTime+1)):
                    E_d.insert(t, [])
                    E_c.insert(t, [])
                    for i, satellite in enumerate(ownSatellites):
                        E_d[t].insert(i, [])
                        E_c[t].insert(i, [])

                        for j, demand in enumerate(demands):
                            stoPen = self.getStoragePenalty(demand.phenomenon, satellite, context, minTime, t)
                            # satellite i stores data for new contract j
                            E_d[t][i].insert(j, lp.addVar(vtype=GRB.BINARY,
                                name='{}-E-{}@{}'.format(satellite.name, demand.name, time)))
                            # penalty for opportunity cost
                            # print "Demand storage penalty:", stoPen
                            J.add(E_d[t][i][j], demand.size*stoPen)
                        for j, contract in enumerate(ownContracts):
                            stoPen = self.getStoragePenalty(contract.demand.phenomenon, satellite, context, minTime, t)
                            # satellite i stores data for contract j
                            E_c[t][i].insert(j, lp.addVar(vtype=GRB.BINARY,
                                name='{}-E-{}@{}'.format(satellite.name, contract.name, time)))
                            # penalty for opportunity cost

                            # print "contract storage penalty: ", stoPen
                            J.add(E_c[t][i][j], contract.demand.size*stoPen)
                        for phenomenon in phenomena:
                            r = LinExpr()
                            for j, demand in enumerate(demands):
                                if phenomenon is None or demand.phenomenon == phenomenon:
                                    r.add(E_d[t][i][j], demand.size)
                            for j, contract in enumerate(ownContracts):
                                if phenomenon is None or contract.demand.phenomenon == phenomenon:
                                    r.add(E_c[t][i][j], contract.demand.size)
                            # constrain data stored in satellite
                            lp.addConstr(r <= satellite.getMaxStored(phenomenon),
                                         '{} max store {} at {}'.format(
                                         satellite.name, phenomenon, time))
                    T_d.insert(t, [])
                    T_c.insert(t, [])
                    for i, satellite in enumerate(allSatellites):
                        T_d[t].insert(i, [])
                        T_c[t].insert(i, [])
                        txLocation = context.propagate(satellite.location, time-context.time)
                        for j, station in enumerate(allStations):
                            owner = context.getElementOwner(station)
                            costSGL = owner.getCost('oSGL')
                            key = '{0}-{1}-{2}'.format(federate, owner, 'oSGL')
                            cost_dict[key] = costSGL
                            T_d[t][i].insert(j, [])
                            T_c[t][i].insert(j, [])
                            rxLocation = context.propagate(station.location, time-context.time)
                            for k, protocol in enumerate(protocolsSGL):
                                T_d[t][i][j].insert(k, [])
                                T_c[t][i][j].insert(k, [])
                                r = LinExpr()
                                maxSize = 0
                                for l, demand in enumerate(demands):
                                    T_d[t][i][j][k].insert(l, lp.addVar(
                                        vtype=GRB.BINARY,
                                        name='{}-T({}/{})-{}@{}'.format(
                                            satellite.name, demand.name,
                                            protocol, station.name, time)))

                                    r.add(T_d[t][i][j][k][l], demand.size)
                                    maxSize = max(maxSize, demand.size if controller.couldTransport(
                                        protocol, demand.generateData(), satellite,
                                        station, txLocation, rxLocation, context)
                                        and not demand.isDefaultedAt(time-context.time) else 0)

                                    if station not in ownStations:
                                        J.add(T_d[t][i][j][k][l],
                                              -1*costSGL*demand.size)
                                        # if maxSize == demand.size:
                                        #     owner.addDemandSignal(federate, protocol, costSGL)

                                for l, contract in enumerate(ownContracts):
                                    T_c[t][i][j][k].insert(l, lp.addVar(
                                        vtype=GRB.BINARY,
                                        name='{}-T({}/{})-{}@{}'.format(
                                            satellite.name, contract.name,
                                            protocol, station.name, time)))

                                    r.add(T_c[t][i][j][k][l], contract.demand.size)
                                    maxSize = max(maxSize, contract.demand.size if controller.couldTransport(
                                        protocol, contract.demand.generateData(), satellite,
                                        station, txLocation, rxLocation, context)
                                        and not contract.demand.isDefaultedAt(
                                        contract.elapsedTime + time-context.time) else 0)

                                    if station not in ownStations:
                                        J.add(T_c[t][i][j][k][l],
                                              -1 * costSGL * contract.demand.size)
                                        # if maxSize == contract.demand.size:
                                        #     owner.addContractSignal(federate, protocol, costSGL)

                                # constrain transmission by visibility
                                lp.addConstr(r <= maxSize, '{}-{} visibility {} at {}'.format(
                                             satellite.name, station.name, protocol, time))
                    for i, satellite in enumerate(allSatellites):
                        for k, protocol in enumerate(protocolsSGL):
                            r = LinExpr()
                            for j, station in enumerate(allStations):
                                for l, demand in enumerate(demands):
                                    r.add(T_d[t][i][j][k][l], demand.size)
                                for l, contract in enumerate(ownContracts):
                                    r.add(T_c[t][i][j][k][l], contract.demand.size)
                            # constrain data transmitted by satellite
                            lp.addConstr(r <= satellite.getMaxTransmitted(protocol)
                                - (satellite.getTransmitted(protocol) if time == minTime else 0),
                                '{} max transmit {} at {}'.format(satellite.name, protocol, time))
                    for j, station in enumerate(allStations):
                        for k, protocol in enumerate(protocolsSGL):
                            r = LinExpr()
                            for i, satellite in enumerate(allSatellites):
                                for l, demand in enumerate(demands):
                                    r.add(T_d[t][i][j][k][l], demand.size)
                                for l, contract in enumerate(ownContracts):
                                    r.add(T_c[t][i][j][k][l], contract.demand.size)
                            # constrain data received by station
                            if station in ownStations:
                                lp.addConstr(r <= station.getMaxReceived(protocol)
                                    - (station.getReceived(protocol) if time == minTime else 0),
                                    '{} max receive {} at {}'.format(station.name, protocol, time))
                            else:
                                # do not assume future availability
                                lp.addConstr(r <= ((station.getMaxReceived(protocol)
                                    - station.getReceived(protocol)) if time == minTime else 0),
                                    '{} max receive {} at {}'.format(station.name, protocol, time))
                    L_d.insert(t, [])
                    L_c.insert(t, [])
                    for i, txSatellite in enumerate(allSatellitesISL):
                        L_d[t].insert(i, [])
                        L_c[t].insert(i, [])
                        txLocation = context.propagate(txSatellite.location, time-context.time)
                        for j, rxSatellite in enumerate(allSatellitesISL):
                            owner = context.getElementOwner(rxSatellite)
                            costISL = owner.getCost('oISL')
                            key = '{0}-{1}-{2}'.format(federate, owner, 'oISL')
                            cost_dict[key] = costISL
                            L_d[t][i].insert(j, [])
                            L_c[t][i].insert(j, [])
                            rxLocation = context.propagate(rxSatellite.location, time-context.time)
                            for k, protocol in enumerate(protocolsISL):
                                L_d[t][i][j].insert(k, [])
                                L_c[t][i][j].insert(k, [])
                                r = LinExpr()
                                maxSize = 0
                                for l, demand in enumerate(demands):
                                    L_d[t][i][j][k].insert(l, lp.addVar(
                                        vtype=GRB.BINARY,
                                        name='{}-T({}/{})-{}@{}'.format(
                                            txSatellite.name, demand.name,
                                            protocol, rxSatellite.name, time)))

                                    r.add(L_d[t][i][j][k][l], demand.size)
                                    maxSize = max(maxSize, demand.size if controller.couldTransport(
                                        protocol, demand.generateData(), txSatellite, rxSatellite,
                                        txLocation, rxLocation, context) and not demand.isDefaultedAt(
                                        time-context.time) else 0)

                                    if (txSatellite not in ownSatellites
                                        or rxSatellite not in ownSatellites):
                                        J.add(L_d[t][i][j][k][l],
                                              -1*costISL*demand.size)
                                        # if maxSize == demand.size:
                                        #     owner.addDemandSignal(federate, protocol, costISL)
                                    else:
                                        # small penalty for opportunity cost
                                        J.add(L_d[t][i][j][k][l], self.islPenalty*demand.size)

                                for l, contract in enumerate(ownContracts):
                                    L_c[t][i][j][k].insert(l, lp.addVar(
                                        vtype=GRB.BINARY,
                                        name='{}-T({}/{})-{}@{}'.format(
                                            txSatellite.name, contract.name,
                                            protocol, rxSatellite.name, time)))

                                    r.add(L_c[t][i][j][k][l], contract.demand.size)
                                    maxSize = max(maxSize, contract.demand.size if controller.couldTransport(
                                        protocol, contract.demand.generateData(),
                                        txSatellite, rxSatellite, txLocation,
                                        rxLocation, context) and not contract.demand.isDefaultedAt(
                                        contract.elapsedTime + time-context.time) else 0)

                                    if (txSatellite not in ownSatellites
                                        or rxSatellite not in ownSatellites):
                                        J.add(L_c[t][i][j][k][l],
                                              -1 * costISL * contract.demand.size)
                                        # if maxSize == contract.demand.size:
                                        #     owner.addContractSignal(federate, protocol, costISL)
                                    else:
                                        # small penalty for opportunity cost
                                        J.add(L_c[t][i][j][k][l],
                                              self.islPenalty * contract.demand.size)
                                # constrain transmission by visibility
                                lp.addConstr(r <= maxSize, '{}-{} visibility {} at {}'.format(
                                    txSatellite.name, rxSatellite.name, protocol, time))
                    for i, txSatellite in enumerate(allSatellitesISL):
                        for k, protocol in enumerate(protocolsISL):
                            r = LinExpr()
                            for j, rxSatellite in enumerate(allSatellitesISL):
                                for l, demand in enumerate(demands):
                                    r.add(L_d[t][i][j][k][l], demand.size)
                                for l, contract in enumerate(ownContracts):
                                    r.add(L_c[t][i][j][k][l], contract.demand.size)
                            # constrain data transmitted by satellite
                            lp.addConstr(r <= (txSatellite.getMaxTransmitted(protocol)
                                - (txSatellite.getTransmitted(protocol) if time == minTime else 0)),
                                '{} max transmit {} at {}'.format(txSatellite.name, protocol, time))
                    for j, rxSatellite in enumerate(allSatellitesISL):
                        for k, protocol in enumerate(protocolsISL):
                            r = LinExpr()
                            for i, txSatellite in enumerate(allSatellitesISL):
                                for l, demand in enumerate(demands):
                                    r.add(L_d[t][i][j][k][l], demand.size)
                                for l, contract in enumerate(ownContracts):
                                    r.add(L_c[t][i][j][k][l], contract.demand.size)
                            if rxSatellite in ownSatellites:
                                # constrain data received by station
                                lp.addConstr(r <= rxSatellite.getMaxReceived(protocol)
                                    - (rxSatellite.getReceived(protocol) if time == minTime else 0),
                                    '{} max receive {} at {}'.format(rxSatellite.name, protocol, time))
                            else:
                                # do not assume future availability
                                lp.addConstr(r <= ((rxSatellite.getMaxReceived(protocol)
                                    - rxSatellite.getReceived(protocol)) if time == minTime else 0),
                                    '{} max receive {} at {}'.format(rxSatellite.name, protocol, time))
                    R_d.insert(t, [])
                    R_c.insert(t, [])
                    for i, element in enumerate(allElements):
                        location = context.propagate(element.location, time-context.time)
                        R_d[t].insert(i, [])
                        R_c[t].insert(i, [])
                        for j, demand in enumerate(demands):
                            R_d[t][i].insert(j, lp.addVar(vtype=GRB.BINARY,
                                name='{}-R-{}@{}'.format(element.name, demand.name, time)))
                            J.add(R_d[t][i][j], demand.getValueAt(time-context.time)
                                  if demand.isCompletedAt(location)
                                  else demand.getDefaultValue())
                        for j, contract in enumerate(ownContracts):
                            R_c[t][i].insert(j, lp.addVar(vtype=GRB.BINARY,
                                name='{}-R-{}@{}'.format(element.name, contract.name, time)))
                            J.add(R_c[t][i][j], contract.demand.getValueAt(
                                contract.elapsedTime + time-context.time)
                                  if contract.demand.isCompletedAt(location)
                                  else contract.demand.getDefaultValue())
                    for i, satellite in enumerate(allSatellites):
                        R_i = allElements.index(satellite)
                        for j, demand in enumerate(demands):
                            r = LinExpr()
                            if satellite in ownSatellites:
                                SE_i = ownSatellites.index(satellite)
                                if time==minTime:
                                    r.add(S[SE_i][j], 1)
                                else:
                                    r.add(E_d[t-1][SE_i][j],1)
                                r.add(E_d[t][SE_i][j],-1)
                            r.add(R_d[t][R_i][j],-1)
                            for k, station in enumerate(allStations):
                                for l, protocol in enumerate(protocolsSGL):
                                    r.add(T_d[t][i][k][l][j],-1)
                            if satellite in allSatellitesISL:
                                isl_i = allSatellitesISL.index(satellite)
                                for k, rxSatellite in enumerate(allSatellitesISL):
                                    for l, protocol in enumerate(protocolsISL):
                                        r.add(L_d[t][isl_i][k][l][j],-1)
                                        r.add(L_d[t][k][isl_i][l][j],1)
                            # constrain net flow of new contracts at each satellite
                            lp.addConstr(r == 0, '{} net flow {} at {}'.format(
                                         satellite.name, demand.name, time))
                        for j, contract in enumerate(ownContracts):
                            r = LinExpr()
                            if satellite in ownSatellites:
                                SE_i = ownSatellites.index(satellite)
                                if time==minTime:
                                    # existing contracts are initial conditions
                                    pass
                                else:
                                    r.add(E_c[t-1][SE_i][j],1)
                                r.add(E_c[t][SE_i][j],-1)
                            r.add(R_c[t][R_i][j],-1)
                            for k, station in enumerate(allStations):
                                for l, protocol in enumerate(protocolsSGL):
                                    r.add(T_c[t][i][k][l][j],-1)
                            if satellite in allSatellitesISL:
                                isl_i = allSatellitesISL.index(satellite)
                                for k, rxSatellite in enumerate(allSatellitesISL):
                                    for l, protocol in enumerate(protocolsISL):
                                        r.add(L_c[t][isl_i][k][l][j],-1)
                                        r.add(L_c[t][k][isl_i][l][j],1)
                            # constrain net flow of contracts at each satellite
                            if satellite in ownSatellites:
                                SE_i = ownSatellites.index(satellite)
                                lp.addConstr(r == -1*(E_c0[SE_i][j] if time == minTime else 0),
                                             '{} net flow {} at {}'.format(
                                             satellite.name, contract.name, time))
                            else:
                                lp.addConstr(r == 0, '{} net flow {} at {}'.format(
                                        satellite.name, contract.name, time))
                    if time+1 > maxTime and self.planningHorizon > 0:
                        for i, satellite in enumerate(ownSatellites):
                            r = LinExpr()
                            for j, demand in enumerate(demands):
                                r.add(E_d[t][i][j],1)
                            for j, contract in enumerate(ownContracts):
                                r.add(E_c[t][i][j],1)
                            # constrain boundary flow of each satellite
                            lp.addConstr(r == 0, '{} boundary flow'.format(satellite.name))
                    for k, station in enumerate(allStations):
                        R_k = allElements.index(station)
                        for j, demand in enumerate(demands):
                            r = LinExpr()
                            r.add(R_d[t][R_k][j],-1)
                            for i, satellite in enumerate(allSatellites):
                                for l, protocol in enumerate(protocolsSGL):
                                    r.add(T_d[t][i][k][l][j],1)
                            # constrain net flow of new contracts at each station
                            lp.addConstr(r == 0, '{} net flow {} at {}'.format(
                                    station.name, demand.name, time))
                        for j, contract in enumerate(ownContracts):
                            r = LinExpr()
                            r.add(R_c[t][R_k][j],-1)
                            for i, satellite in enumerate(allSatellites):
                                for l, protocol in enumerate(protocolsSGL):
                                    r.add(T_c[t][i][k][l][j],1)
                            # constrain net flow of contracts at each station
                            lp.addConstr(r == 0, '{} net flow {} at {}'.format(
                                    station.name, contract.name, time))
                r = LinExpr()
                for l, demand in enumerate(demands):
                    for i, element in enumerate(allElements):
                        location = context.propagate(element.location, time-context.time)
                        r.add(R_d[0][i][l], (demand.getValueAt(0)
                            if demand.isCompletedAt(location)
                            else demand.getDefaultValue()))
                    for i, satellite in enumerate(allSatellites):
                        for j, station in enumerate(allStations):
                            for k, protocol in enumerate(protocolsSGL):
                                if station not in ownStations:
                                    r.add(T_d[0][i][j][k][l], -1*self.costSGL*demand.size)
                    for i, txSatellite in enumerate(allSatellitesISL):
                        for j, rxSatellite in enumerate(allSatellitesISL):
                            for k, protocol in enumerate(protocolsISL):
                                if rxSatellite not in ownSatellites:
                                    r.add(L_d[0][i][j][k][l], -1*self.costISL*demand.size)
                for l, contract in enumerate(ownContracts):
                    for i, element in enumerate(allElements):
                        location = context.propagate(element.location, time-context.time)
                        r.add(R_c[0][i][l], (contract.demand.getValueAt(contract.elapsedTime)
                            if contract.demand.isCompletedAt(location)
                            else contract.demand.getDefaultValue()))
                    for i, satellite in enumerate(allSatellites):
                        for j, station in enumerate(allStations):
                            for k, protocol in enumerate(protocolsSGL):
                                if station not in ownStations:
                                    r.add(T_c[0][i][j][k][l],
                                          -1*self.costSGL*contract.demand.size)
                    for i, txSatellite in enumerate(allSatellitesISL):
                        for j, rxSatellite in enumerate(allSatellitesISL):
                            for k, protocol in enumerate(protocolsISL):
                                if rxSatellite not in ownSatellites:
                                    r.add(L_c[0][i][j][k][l],
                                          -1*self.costISL*contract.demand.size)
                lp.addConstr(r >= -1 - federate.cash,
                             '{} net cash must be positive'.format(federate.name))

                lp.setObjective(J, GRB.MAXIMIZE)
                lp.setParam('OutputFlag', False)
                # print R_d
                lp.optimize()


                def _transportContract(operations, satellite, contract, context):
                    i = allSatellites.index(satellite)
                    R_i = allElements.index(satellite)
                    j = ownContracts.index(contract)
                    data = context.getData(contract)
                    if data is not None:
                        if R_c[0][R_i][j].x > 0:
                            controller.resolve(contract, context)
                        elif (satellite in ownSatellites
                              and E_c[0][ownSatellites.index(satellite)][j].x > 0):
                            satellite.store(data)
                        elif any(any(T_c[0][i][k][l][j].x
                                     for k, station in enumerate(allStations))
                                 for l, protocol in enumerate(protocolsSGL)):
                            for k, station in enumerate(allStations):
                                for l, protocol in enumerate(protocolsSGL):
                                    if(T_c[0][i][k][l][j].x):
                                        controller.transport(protocol, data, satellite,
                                                             station, context)
                                        controller.resolve(contract, context)
                                        if station not in ownStations:
                                            owner = context.getElementOwner(station)
                                            owner.addThirdContract(federate, protocol, cost_dict['{0}-{1}-{2}'.format(federate, owner, protocol)])
                                            # print "transport contract: from sattelite to other's station"
                                            supplier = context.getElementOwner(station)
                                            controller.exchange(costSGL,
                                                                federate, supplier)
                                            logging.debug('{} paid {} to {} for SGL'.format(
                                                federate.name, costSGL, supplier.name))

                        elif satellite in allSatellitesISL:
                            isl_i = allSatellitesISL.index(satellite)
                            for k, rxSatellite in enumerate(allSatellitesISL):
                                for l, protocol in enumerate(protocolsISL):
                                    if(L_c[0][isl_i][k][l][j].x):
                                        controller.transport(protocol, data, satellite,
                                                             rxSatellite, context)
                                        _transportContract(operations, rxSatellite,
                                                           contract, context)
                                        if rxSatellite not in ownSatellites:
                                            owner = context.getElementOwner(rxSatellite)
                                            owner.addThirdContract(federate, protocol, cost_dict['{0}-{1}-{2}'.format(federate, owner, protocol)])
                                            # print "transport contract: from sattelite to other's sattelite"
                                            supplier = context.getElementOwner(rxSatellite)
                                            controller.exchange(costISL, federate, supplier)
                                            logging.debug('{} paid {} to {} for ISL'.format(
                                                federate.name, costISL, supplier.name))

                def _transportDemand(operations, satellite, demand, context):
                    i = allSatellites.index(satellite)
                    R_i = allElements.index(satellite)
                    j = demands.index(demand)
                    contract = context.getContract(demand)
                    data = context.getData(contract)
                    if contract is not None and data is not None:
                        if R_d[0][R_i][j].x > 0:
                            controller.resolve(contract, context)
                        elif (satellite in ownSatellites
                              and E_d[0][ownSatellites.index(satellite)][j].x > 0):
                            satellite.store(data)
                        elif any(any(T_d[0][i][k][l][j].x
                                     for k, station in enumerate(allStations))
                                 for l, protocol in enumerate(protocolsSGL)):
                            for k, station in enumerate(allStations):
                                for l, protocol in enumerate(protocolsSGL):
                                    if(T_d[0][i][k][l][j].x):
                                        controller.transport(protocol, data, satellite,
                                                             station, context)
                                        controller.resolve(contract, context)
                                        if station not in ownStations:
                                            owner = context.getElementOwner(station)
                                            costSGL = owner.getCost('oSGL')
                                            key = '{0}-{1}-{2}'.format(federate, owner, protocol)
                                            # print key in cost_dict
                                            owner.addThirdDemand(federate, protocol, cost_dict[key])
                                            federate.addIssueDemand(owner, protocol, cost_dict[key])
                                            # print "transport demand: from sattelite to other's station"
                                            supplier = context.getElementOwner(station)
                                            controller.exchange(costSGL,
                                                                federate, supplier)
                                            logging.debug('{} paid {} to {} for SGL' .format(
                                                federate.name, costSGL, supplier.name))
                        elif satellite in allSatellitesISL:
                            isl_i = allSatellitesISL.index(satellite)
                            for k, rxSatellite in enumerate(allSatellitesISL):
                                for l, protocol in enumerate(protocolsISL):
                                    if(L_d[0][isl_i][k][l][j].x):
                                        controller.transport(protocol, data, satellite,
                                                             rxSatellite, context)
                                        _transportDemand(operations, rxSatellite,
                                                         demand, context)
                                        if rxSatellite not in ownSatellites:
                                            owner = context.getElementOwner(rxSatellite)
                                            costISL = owner.getCost('oISL')
                                            key = '{0}-{1}-{2}'.format(federate, owner, protocol)
                                            # print key in cost_dict
                                            owner.addThirdDemand(federate, protocol, cost_dict[key])
                                            federate.addIssueDemand(owner, protocol, cost_dict[key])
                                            # print "transport demand: from sattelite to other's sattelite"
                                            supplier = context.getElementOwner(rxSatellite)
                                            controller.exchange(costISL,
                                                                federate, supplier)
                                            logging.debug('{} paid {} to {} for ISL' .format(
                                                federate.name, costISL, supplier.name))

                # print "Before cash:", federate.cash
                # first, transport contracts to resolution
                for j, contract in enumerate(ownContracts):
                    if any(R_c[0][i][j].x > 0 for i, element in enumerate(allElements)):
                        logging.debug('Transporting contract {} for resolution...'.format(contract.name))
                        satellite = context.getDataElement(contract)
                        _transportContract(self, satellite, contract, context)

                # second, sense and transport demands to resolution
                for j, demand in enumerate(demands):
                    if any(R_d[0][i][j].x > 0 for i, element in enumerate(allElements)):
                        logging.debug('Sensing and transporting demand {} for resolution...'.format(demand.name))
                        satellite = next(e for i, e in enumerate(ownSatellites) if S[i][j].x > 0)
                        contract = federate.contract(demand, context)
                        federate.senseAndStore(contract, satellite, context)
                        _transportDemand(self, satellite, demand, context)

                # third, sense all demands to be stored
                for j, demand in enumerate(demands):
                    if (all(R_d[0][i][j].x < 1 for i, element in enumerate(allElements))
                        and any(S[i][j].x > 0 for i, element in enumerate(ownSatellites))):
                        logging.debug('Sensing demand {} for storage...'.format(demand.name))
                        satellite = next(e for i, e in enumerate(ownSatellites) if S[i][j].x > 0)
                        contract = federate.contract(demand, context)
                        federate.senseAndStore(contract, satellite, context)

                # fourth, transport demands to storage
                for j, demand in enumerate(demands):
                    if (all(R_d[0][i][j].x < 1 for i, element in enumerate(allElements))
                        and any(S[i][j].x > 0 for i, element in enumerate(ownSatellites))):
                        logging.debug('Transporting demand {} for storage...'.format(demand.name))
                        satellite = next(e for i, e in enumerate(ownSatellites) if S[i][j].x > 0)
                        _transportDemand(self, satellite, demand, context)

                # finally, transport contracts to storage
                for j, contract in enumerate(ownContracts):
                    if all(R_c[0][i][j].x < 1 for i, element in enumerate(allElements)):
                        logging.debug('Transporting contract {} for storage...'.format(contract.name))
                        satellite = context.getDataElement(contract)
                        _transportContract(self, satellite, contract, context)

                # print "After cash: ", federate.cash
                # contracts_demands = federate.getthirdcontractsdemands()
                # print "The third contracts and demands: (contract signals, demand signals, real contract, real demand)", self.costSGL
                # for c in contracts_demands:
                #     if c:
                #         print c

            except GurobiError as e:
                print('Error code ' + str(e.errno) + ": " + str(e))
            except AttributeError as e:
                print str(e)
                print('Encountered an attribute error')


