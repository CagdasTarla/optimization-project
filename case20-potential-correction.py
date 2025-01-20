# -*- : utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""



plants = ['istanbul', 'ankara', 'izmir']
components = ['aluminum frames', 'carbon fiber frames', 'manual modules', 'advanced control modules', 'advanced sensor modules' ]
component_robotic = ['robotic kit']
months= ['month 1', 'month 2']
labor_required = {
('istanbul','aluminum frames','month 1'): 1, ('istanbul','carbon fiber frames','month 1'):1.5, ('istanbul','manual modules','month 1'):1.5, ('istanbul', 'advanced control modules','month 1'):3, ('istanbul','advanced sensor modules','month 1'):4, 
('ankara','aluminum frames','month 1'): 3.5, ('ankara','carbon fiber frames','month 1'):3.5, ('ankara','manual modules','month 1'):4.5, ('ankara', 'advanced control modules','month 1'):4, ('ankara','advanced sensor modules','month 1'):5,
('izmir','aluminum frames','month 1'): 3, ('izmir','carbon fiber frames','month 1'):3.5, ('izmir','manual modules','month 1'):4, ('izmir', 'advanced control modules','month 1'):4.5, ('izmir','advanced sensor modules','month 1'):5.5,
('istanbul','aluminum frames','month 2'): 1, ('istanbul','carbon fiber frames','month 2'):1.5, ('istanbul','manual modules','month 2'):1.5, ('istanbul', 'advanced control modules','month 2'):3, ('istanbul','advanced sensor modules','month 2'):4, 
('ankara','aluminum frames','month 2'): 3.5, ('ankara','carbon fiber frames','month 2'):3.5, ('ankara','manual modules','month 2'):4.5, ('ankara', 'advanced control modules','month 2'):4, ('ankara','advanced sensor modules','month 2'):5,
('izmir','aluminum frames','month 2'): 3, ('izmir','carbon fiber frames','month 2'):3.5, ('izmir','manual modules','month 2'):4, ('izmir', 'advanced control modules','month 2'):4.5, ('izmir','advanced sensor modules','month 2'):5.5}             

packing_required ={
('istanbul','aluminum frames','month 1'): 4, ('istanbul','carbon fiber frames','month 1'):4, ('istanbul','manual modules','month 1'):5, ('istanbul', 'advanced control modules','month 1'):6, ('istanbul','advanced sensor modules','month 1'):6, 
('ankara','aluminum frames','month 1'): 7, ('ankara','carbon fiber frames','month 1'):7, ('ankara','manual modules','month 1'):8, ('ankara', 'advanced control modules','month 1'):9, ('ankara','advanced sensor modules','month 1'):7,
('izmir','aluminum frames','month 1'): 7.5, ('izmir','carbon fiber frames','month 1'):7.5, ('izmir','manual modules','month 1'):8.5, ('izmir', 'advanced control modules','month 1'):8.5, ('izmir','advanced sensor modules','month 1'):8,
('istanbul','aluminum frames','month 2'): 4, ('istanbul','carbon fiber frames','month 2'):4, ('istanbul','manual modules','month 2'):5, ('istanbul', 'advanced control modules','month 2'):6, ('istanbul','advanced sensor modules','month 2'):6, 
('ankara','aluminum frames','month 2'): 7, ('ankara','carbon fiber frames','month 2'):7, ('ankara','manual modules','month 2'):8, ('ankara', 'advanced control modules','month 2'):9, ('ankara','advanced sensor modules','month 2'):7,
('izmir','aluminum frames','month 2'): 7.5, ('izmir','carbon fiber frames','month 2'):7.5, ('izmir','manual modules','month 2'):8.5, ('izmir', 'advanced control modules','month 2'):8.5, ('izmir','advanced sensor modules','month 2'):8}            

cmin_demand= {('istanbul','aluminum frames','month 1'): 0, ('istanbul','carbon fiber frames','month 1'):100, ('istanbul','manual modules','month 1'):200, ('istanbul', 'advanced control modules','month 1'):30, ('istanbul','advanced sensor modules','month 1'):100, 
        ('ankara','aluminum frames','month 1'): 0, ('ankara','carbon fiber frames','month 1'):100, ('ankara','manual modules','month 1'):200, ('ankara', 'advanced control modules','month 1'):30, ('ankara','advanced sensor modules','month 1'):100, 
        ('izmir','aluminum frames','month 1'): 0, ('izmir','carbon fiber frames','month 1'):50, ('izmir','manual modules','month 1'):100, ('izmir', 'advanced control modules','month 1'):15, ('izmir','advanced sensor modules','month 1'):100, 
        ('istanbul','aluminum frames','month 2'): 0, ('istanbul','carbon fiber frames','month 2'):100, ('istanbul','manual modules','month 2'):200, ('istanbul', 'advanced control modules','month 2'):30, ('istanbul','advanced sensor modules','month 2'):100, 
('ankara','aluminum frames','month 2'): 0, ('ankara','carbon fiber frames','month 2'):100, ('ankara','manual modules','month 2'):200, ('ankara', 'advanced control modules','month 2'):30, ('ankara','advanced sensor modules','month 2'):100, 
('izmir','aluminum frames','month 2'): 0, ('izmir','carbon fiber frames','month 2'):50, ('izmir','manual modules','month 2'):100, ('izmir', 'advanced control modules','month 2'):15, ('izmir','advanced sensor modules','month 2'):100}                                                                  

cmin_demand_robotic = {('istanbul','robotic kit','month 1'):0, ('ankara','robotic kit','month 1'):0, ('izmir','robotic kit','month 1'):0,
                       ('istanbul','robotic kit','month 2'):0,('ankara','robotic kit','month 2'):0, ('izmir','robotic kit','month 2'):0}
                                                                                                                       
cmax_demand = {
        ('istanbul','aluminum frames','month 1'): 2000, ('istanbul','carbon fiber frames','month 1'):2000, ('istanbul','manual modules','month 1'):2000, ('istanbul', 'advanced control modules','month 1'):2000, ('istanbul','advanced sensor modules','month 1'):2000, 
        ('ankara','aluminum frames','month 1'): 2000, ('ankara','carbon fiber frames','month 1'):2000, ('ankara','manual modules','month 1'):2000, ('ankara', 'advanced control modules','month 1'):2000, ('ankara','advanced sensor modules','month 1'):2000, 
        ('izmir','aluminum frames','month 1'): 2000, ('izmir','carbon fiber frames','month 1'):2000, ('izmir','manual modules','month 1'):2000, ('izmir', 'advanced control modules','month 1'):2000, ('izmir','advanced sensor modules','month 1'):2000, 
        ('istanbul','aluminum frames','month 2'): 2000, ('istanbul','carbon fiber frames','month 2'):2000, ('istanbul','manual modules','month 2'):2000, ('istanbul', 'advanced control modules','month 2'):2000, ('istanbul','advanced sensor modules','month 2'):2000, 
('ankara','aluminum frames','month 2'): 2000, ('ankara','carbon fiber frames','month 2'):2000, ('ankara','manual modules','month 2'):2000, ('ankara', 'advanced control modules','month 2'):2000, ('ankara','advanced sensor modules','month 2'):2000, 
('izmir','aluminum frames','month 2'): 2000, ('izmir','carbon fiber frames','month 2'):2000, ('izmir','manual modules','month 2'):2000, ('izmir', 'advanced control modules','month 2'):2000, ('izmir','advanced sensor modules','month 2'):2000}      

cmax_demand_robotic = {('istanbul','robotic kit','month 1'):200,('ankara','robotic kit','month 1'):200,('izmir','robotic kit','month 1'):200,('istanbul','robotic kit','month 2'):200,
                       ('ankara','robotic kit','month 2'):200, ('izmir','robotic kit','month 2'):200}

available_labor_hours={('istanbul', 'month 1'):12000,('istanbul', 'month 2'):12000,('ankara', 'month 1'):15000,('ankara', 'month 2'):15000,('izmir', 'month 1'):22000,('izmir', 'month 2'):22000 }
available_packing_hours={('istanbul', 'month 1'):20000,('istanbul', 'month 2'):20000,('ankara', 'month 1'):40000,('ankara', 'month 2'):40000,('izmir', 'month 1'):35000,('izmir', 'month 2'):35000 }

assembly_per_set = {('istanbul','month 1'):65, ('ankara','month 1'):60,('izmir','month 1'):65,
                    ('istanbul','month 2'):65, ('ankara','month 2'):60,('izmir','month 2'):65} 

assembly_total = {('istanbul','month 1'):5500, ('ankara','month 1'):5000,('izmir','month 1'):6000,
                  ('istanbul','month 2'):5500, ('ankara','month 2'):5000,('izmir','month 2'):6000}      

robotic_kit_req={'aluminum frames':13, 'carbon fiber frames':13, 'manual modules':10, 'advanced control modules':3, 'advanced sensor modules':3}
           
production_costs={('istanbul','aluminum frames', 'month 1'): 6, ('istanbul','carbon fiber frames', 'month 1'):19, ('istanbul','manual modules', 'month 1'):4, ('istanbul', 'advanced control modules', 'month 1'):10, ('istanbul','advanced sensor modules', 'month 1'):26, 
            ('ankara','aluminum frames', 'month 1'): 5, ('ankara','carbon fiber frames', 'month 1'):18, ('ankara','manual modules', 'month 1'):5, ('ankara', 'advanced control modules', 'month 1'):11, ('ankara','advanced sensor modules', 'month 1'):24, 
        ('izmir','aluminum frames', 'month 1'): 7, ('izmir','carbon fiber frames', 'month 1'):20, ('izmir','manual modules', 'month 1'):5, ('izmir', 'advanced control modules', 'month 1'):12, ('izmir','advanced sensor modules', 'month 1'):27,
        ('istanbul','aluminum frames', 'month 2'): 6*1.12, ('istanbul','carbon fiber frames', 'month 2'):19*1.12, ('istanbul','manual modules', 'month 2'):4*1.12, ('istanbul', 'advanced control modules', 'month 2'):10*1.12, ('istanbul','advanced sensor modules', 'month 2'):26*1.12, 
        ('ankara','aluminum frames', 'month 2'): 5*1.12, ('ankara','carbon fiber frames', 'month 2'):18*1.12, ('ankara','manual modules', 'month 2'):5*1.12, ('ankara', 'advanced control modules', 'month 2'):11*1.12, ('ankara','advanced sensor modules', 'month 2'):24*1.12, 
        ('izmir','aluminum frames', 'month 2'): 7*1.12, ('izmir','carbon fiber frames', 'month 2'):20*1.12, ('izmir','manual modules', 'month 2'):5*1.12, ('izmir', 'advanced control modules', 'month 2'):12*1.12, ('izmir','advanced sensor modules', 'month 2'):27*1.12}          

production_costs_robotic ={('istanbul','robotic kit', 'month 1'):178,('ankara','robotic kit', 'month 1'):175, ('izmir','robotic kit', 'month 1'):180,
                           ('istanbul','robotic kit', 'month 2'):178*1.12,('ankara','robotic kit', 'month 2'):175*1.12, ('izmir','robotic kit', 'month 2'):180*1.12}
inventory_costs={
    ('istanbul','aluminum frames', 'month 1'): 6*0.08, ('istanbul','carbon fiber frames', 'month 1'):19*0.08, ('istanbul','manual modules', 'month 1'):4*0.08, ('istanbul', 'advanced control modules', 'month 1'):10*0.08, ('istanbul','advanced sensor modules', 'month 1'):26*0.08, 
    ('ankara','aluminum frames', 'month 1'): 5*0.08, ('ankara','carbon fiber frames', 'month 1'):18*0.08, ('ankara','manual modules', 'month 1'):5*0.08, ('ankara', 'advanced control modules', 'month 1'):11*0.08, ('ankara','advanced sensor modules', 'month 1'):24*0.08, 
    ('izmir','aluminum frames', 'month 1'): 7*0.08, ('izmir','carbon fiber frames', 'month 1'):20*0.08, ('izmir','manual modules', 'month 1'):5*0.08, ('izmir', 'advanced control modules', 'month 1'):12*0.08, ('izmir','advanced sensor modules', 'month 1'):27*0.08, 
    ('istanbul','aluminum frames', 'month 2'): 6*.12*0.08, ('istanbul','carbon fiber frames', 'month 2'):19*1.12*0.08, ('istanbul','manual modules', 'month 2'):4*1.12*0.08, ('istanbul', 'advanced control modules', 'month 2'):10*1.12*0.08, ('istanbul','advanced sensor modules', 'month 2'):26*1.12*0.08, 
    ('ankara','aluminum frames', 'month 2'): 5*1.12*0.08, ('ankara','carbon fiber frames', 'month 2'):18*1.12*0.08, ('ankara','manual modules', 'month 2'):5*1.12*0.08, ('ankara', 'advanced control modules', 'month 2'):11*1.12*0.08, ('ankara','advanced sensor modules', 'month 2'):24*1.12*0.08, 
    ('izmir','aluminum frames', 'month 2'): 7*1.12*0.08, ('izmir','carbon fiber frames', 'month 2'):20*1.12*0.08, ('izmir','manual modules', 'month 2'):5*1.12*0.08, ('izmir', 'advanced control modules', 'month 2'):12*1.12*0.08, ('izmir','advanced sensor modules', 'month 2'):27*1.12*0.08} 

inventory_costs_robotic ={('istanbul','robotic kit', 'month 1'):178*0.08,('ankara','robotic kit', 'month 1'):175*0.08,('izmir','robotic kit', 'month 1'):180*0.08,
                          ('istanbul','robotic kit', 'month 2'):178*1.12*0.08,('ankara','robotic kit', 'month 2'):175*1.12*0.08,('izmir','robotic kit', 'month 2'):180*1.12*0.08}

selling_price={('istanbul','aluminum frames', 'month 1'): 10, ('istanbul','carbon fiber frames', 'month 1'):25, ('istanbul','manual modules', 'month 1'):8, ('istanbul', 'advanced control modules', 'month 1'):18, ('istanbul','advanced sensor modules', 'month 1'):40, 
            ('ankara','aluminum frames', 'month 1'): 10, ('ankara','carbon fiber frames', 'month 1'):25, ('ankara','manual modules', 'month 1'):8, ('ankara', 'advanced control modules', 'month 1'):18, ('ankara','advanced sensor modules', 'month 1'):40, 
        ('izmir','aluminum frames', 'month 1'): 12, ('izmir','carbon fiber frames', 'month 1'):30, ('izmir','manual modules', 'month 1'):10, ('izmir', 'advanced control modules', 'month 1'):22, ('izmir','advanced sensor modules', 'month 1'):45, 
        ('istanbul','aluminum frames', 'month 2'): 10, ('istanbul','carbon fiber frames', 'month 2'):25, ('istanbul','manual modules', 'month 2'):8, ('istanbul', 'advanced control modules', 'month 2'):18, ('istanbul','advanced sensor modules', 'month 2'):40, 
                    ('ankara','aluminum frames', 'month 2'): 10, ('ankara','carbon fiber frames', 'month 2'):25, ('ankara','manual modules', 'month 2'):8, ('ankara', 'advanced control modules', 'month 2'):18, ('ankara','advanced sensor modules', 'month 2'):40, 
                ('izmir','aluminum frames', 'month 2'): 12, ('izmir','carbon fiber frames', 'month 2'):30, ('izmir','manual modules', 'month 2'):10, ('izmir', 'advanced control modules', 'month 2'):22, ('izmir','advanced sensor modules', 'month 2'):45}      

selling_price_robotic ={('istanbul','robotic kit', 'month 1'):290,('ankara','robotic kit', 'month 1'):290,('izmir','robotic kit', 'month 1'):310,
                        ('istanbul','robotic kit', 'month 2'):290,('ankara','robotic kit', 'month 2'):290, ('izmir','robotic kit', 'month 2'):310}

import numpy as np 
import pandas as pd
import pyomo.environ as pyo
from pyomo.opt import SolverFactory               

#Construct the model
mdl = pyo.ConcreteModel('PO_Model') 
#Define sets
mdl.robotic_kit = pyo.Set(initialize=component_robotic, doc='Set containing only the robotic kit component')
mdl.I = pyo.Set(initialize=plants, doc='Set of Plants' ) 
mdl.J = pyo.Set(initialize=components, doc='Set of components') 
mdl.T = pyo.Set(initialize=months, doc='time periods')                 
#Define parameters
mdl.pL = pyo.Param(mdl.I, mdl.J,mdl.T, initialize= labor_required, doc='required labor per unit in minutes')           
mdl.pP = pyo.Param(mdl.I, mdl.J,mdl.T, initialize= packing_required, doc='required packing per unit in minutes')                 
mdl.pA = pyo.Param(mdl.I, mdl.T, initialize= assembly_per_set, doc='required time for a robotic kit assembled in city i in month t')
mdl.pJ = pyo.Param(mdl.J, initialize= robotic_kit_req, doc='amount of required component j for a robotic kit')
mdl.pD = pyo.Param(mdl.I, mdl.J, mdl.T, initialize= cmin_demand, doc='minimum demand for componenj in city i in month t')
mdl.pE = pyo.Param(mdl.I, mdl.J, mdl.T, initialize= cmax_demand, doc='maximum demand for component j in city i in month t')
mdl.pC = pyo.Param(mdl.I, mdl.J, mdl.T, initialize= production_costs, doc='production costs for component j in city i in month t ')
mdl.pI = pyo.Param(mdl.I, mdl.J, mdl.T, initialize=inventory_costs, doc='inventory cost for component j in city i at the end of month t')
mdl.pS = pyo.Param(mdl.I, mdl.J, mdl.T, initialize= selling_price, doc='selling price for component j in city i in month t')
mdl.pV = pyo.Param(mdl.I, mdl.T, initialize= available_labor_hours, doc='available labor hours in city i in month t')
mdl.pW = pyo.Param(mdl.I, mdl.T, initialize= available_packing_hours, doc='available packing hours in city i in month t')
mdl.pB = pyo.Param(mdl.I, mdl.T, initialize= assembly_total, doc='total assembly time for robotic kit produced in city i in month t')
mdl.pD_robotic = pyo.Param(mdl.I, mdl.T, initialize={(i, t): cmin_demand_robotic[(i, 'robotic kit', t)] for i in mdl.I for t in mdl.T}, doc='Minimum demand for robotic kits')
mdl.pE_robotic = pyo.Param(mdl.I, mdl.T, initialize={(i, t): cmax_demand_robotic[(i, 'robotic kit', t)] for i in mdl.I for t in mdl.T}, doc='Maximum demand for robotic kits')
mdl.pC_robotic = pyo.Param(mdl.I, mdl.T, initialize={(i, t): production_costs_robotic[(i, 'robotic kit', t)] for i in mdl.I for t in mdl.T}, doc='Production cost for robotic kits')
mdl.pI_robotic = pyo.Param(mdl.I, mdl.T, initialize={(i, t): inventory_costs_robotic[(i, 'robotic kit', t)] for i in mdl.I for t in mdl.T}, doc='Inventory cost for robotic kits')
mdl.pS_robotic = pyo.Param(mdl.I, mdl.T, initialize={(i, t): selling_price_robotic[(i, 'robotic kit', t)] for i in mdl.I for t in mdl.T}, doc='Selling price for robotic kits')



#Define variables
mdl.vX = pyo.Var (mdl.I, mdl.J,mdl.T,  doc='amount of component j produced in city i in month t', 
within=pyo.NonNegativeReals)
mdl.vY = pyo.Var (mdl.I, mdl.T,  doc='amount of robotic kit produced in city i in month t', 
within=pyo.NonNegativeReals)
mdl.vZ = pyo.Var (mdl.I, mdl.J,mdl.T,  doc='amount of component j produced in city i in month t in inventory', 
within=pyo.NonNegativeReals)
mdl.vM=pyo.Var (mdl.I, mdl.J,mdl.T,  doc='amount of component j demanded in city i in month t', 
within=pyo.NonNegativeReals)
mdl.vZ_robotic = pyo.Var(mdl.I, mdl.T, within=pyo.NonNegativeReals, doc='Inventory of Robotic Kits in each plant and month')
mdl.vM_robotic = pyo.Var(mdl.I, mdl.T, within=pyo.NonNegativeReals, doc='Demand for Robotic Kits in each plant and month')

#Define constraints
# labor  capacity
def eLaborCap(mdl, i, t):
    return sum(mdl.vX[i, j, t] * mdl.pL[i, j, t] for j in mdl.J if j != 'robotic kit') <= mdl.pV[i, t]
mdl.eLaborCap = pyo.Constraint(mdl.I,mdl.T, rule=eLaborCap, doc='Labor Capacity constraint')

def ePackingCap(mdl, i, t):
    return sum(mdl.vX[i, j, t] * mdl.pP[i, j, t] for j in mdl.J if j != 'robotic kit') <= mdl.pW[i, t]
mdl.ePackingCap = pyo.Constraint(mdl.I,mdl.T, rule=ePackingCap, doc='Packing Capacity constraint')

def eCarbonFiber(mdl, j, t):
    if j == 'carbon fiber frames':
        return sum(mdl.vX[i, j, t] * 0.25 for i in mdl.I) <= 1000
    else:
        return pyo.Constraint.NoConstraint
mdl.eCarbonFiber= pyo.Constraint(mdl.J,mdl.T, rule=eCarbonFiber, doc='Avalable carbon fiber constraint')    

def eMinDemand(mdl, i, j, t):
    return mdl.vX[i, j, t] - mdl.vZ[i, j, t] >= mdl.pD[i, j, t]

mdl.eMinDemand = pyo.Constraint(mdl.I, mdl.J, mdl.T, rule=eMinDemand, doc='Minimum Demand Constraint')


def eMaxDemand(mdl, i, j, t):
    return mdl.vX[i, j, t] - mdl.vZ[i, j, t] <= mdl.pE[i, j, t]

mdl.eMaxDemand = pyo.Constraint(mdl.I, mdl.J, mdl.T, rule=eMaxDemand, doc='Maximum Demand Constraint')

def eMinDemandRobotic(mdl, i, t):
    return mdl.vM_robotic[i, t] >= mdl.pD_robotic[i, t]

mdl.eMinDemandRobotic = pyo.Constraint(
    mdl.I, mdl.T,
    rule=eMinDemandRobotic,
    doc='Minimum demand constraint for robotic kits'
)

def eMaxDemandRobotic(mdl, i, t):
    return mdl.vM_robotic[i, t] <= mdl.pE_robotic[i, t]

mdl.eMaxDemandRobotic = pyo.Constraint(
    mdl.I, mdl.T,
    rule=eMaxDemandRobotic,
    doc='Maximum demand constraint for robotic kits'
)


def eInventoryBalanceRobotic(mdl, i, t):
    if t == 'month 1':
        return mdl.vZ_robotic[i, t] == mdl.vY[i, t] - mdl.vM_robotic[i, t]
    else:
        return mdl.vZ_robotic[i, t] == mdl.vZ_robotic[i, 'month 1'] + mdl.vY[i, t] - mdl.vM_robotic[i, t]

mdl.eInventoryBalanceRobotic = pyo.Constraint(
    mdl.I, mdl.T,
    rule=eInventoryBalanceRobotic,
    doc='Inventory Balance Constraint for Robotic Kits'
)



def eInventoryBalance(mdl, i, j, t):
    if t == 'month 1':
        # Initial month balance (no previous inventory)
        return mdl.vZ[i, j, t] == mdl.vX[i, j, t] - mdl.vM[i, j, t]
    elif t == 'month 2':
        # Balance from previous month
        return mdl.vZ[i, j, t] == mdl.vZ[i, j, 'month 1'] + mdl.vX[i, j, t] - mdl.vM[i, j, t]
    else:
        return pyo.Constraint.Skip

mdl.eInventoryBalance = pyo.Constraint(mdl.I, mdl.J, mdl.T, rule=eInventoryBalance, doc='Inventory Balance Constraint')

    
def eTotalAssemblyRobotic(mdl, i, t):
    if 'robotic kit' in mdl.J:
        return mdl.vX[i, 'robotic kit', t] * mdl.pA[i, t] <= mdl.pB[i, t]
    else:
        return pyo.Constraint.NoConstraint

mdl.eTotalAssemblyRobotic = pyo.Constraint(
    mdl.I, mdl.T,
    rule=eTotalAssemblyRobotic,
    doc='Total Assembly Constraint for Robotic Kit')

def oTotal_Return(mdl):
    # Total Production and Inventory Costs
    cost = sum(
        mdl.vX[i, j, t] * mdl.pC[i, j, t] + mdl.vZ[i, j, t] * mdl.pI[i, j, t]
        for i in mdl.I for j in mdl.J for t in mdl.T
    ) + sum(
        mdl.vY[i, t] * mdl.pC_robotic[i, t] + mdl.vZ_robotic[i, t] * mdl.pI_robotic[i, t]
        for i in mdl.I for t in mdl.T
    )
    
    # Total Revenue from Components
    revenue = sum(
        (mdl.vX[i, j, t] - mdl.vZ[i, j, t] - mdl.vY[i, t] * robotic_kit_req[j]) * mdl.pS[i, j, t]
        for i in mdl.I for j in mdl.J for t in mdl.T
    )
    
    # Total Revenue from Robotic Kits
    revenue_robotic = sum(
        mdl.vM_robotic[i, t] * mdl.pS_robotic[i, t]
        for i in mdl.I for t in mdl.T
    )
    
    return revenue + revenue_robotic - cost

mdl.oTotal_Return = pyo.Objective(
    rule=oTotal_Return,
    sense=pyo.maximize,
    doc='Total Return Objective Including Robotic Kit Parameters'
)












# Solve the model
Solver = SolverFactory('glpk')
Solver.options['ranges'] = 'C:\\Users\\e257678\\Desktop\\case20(correction)\\SA_Report.txt'
SolverResults = Solver.solve(mdl, tee=True)
SolverResults.write()

# Verify Solver Status
if SolverResults.solver.status != pyo.SolverStatus.ok:
    raise Exception(f"Solver did not exit normally: {SolverResults.solver.status}")

if SolverResults.solver.termination_condition != pyo.TerminationCondition.optimal:
    raise Exception(f"Solver did not find an optimal solution: {SolverResults.solver.termination_condition}")

# Export results
mdl.write('C:\\Users\\e257678\\Desktop\\case20(correction)\\mdl_labels.lp', io_options={'symbolic_solver_labels': True})
mdl.write('C:\\Users\\e257678\\Desktop\\case20(correction)\\mdl_nolabels.lp', io_options={'symbolic_solver_labels': False})

# Save detailed results to file
with open('C:\\Users\\e257678\\Desktop\\case20(correction)\\output_results.txt', 'w') as f:
    mdl.pprint(ostream=f)
    mdl.vX.display(ostream=f)
    mdl.vZ.display(ostream=f)
    mdl.oTotal_Return.display(ostream=f)

# Sensitivity Analysis
import pyomo_sens_analysis_v2 as pyo_SA
pyo_SA.reorganize_SA_report(
    file_path_SA='C:\\Users\\e257678\\Desktop\\case20(correction)\\SA_Report.txt',
    file_path_LP_labels='C:\\Users\\e257678\\Desktop\\case20(correction)\\mdl_labels.lp',
    file_path_LP_nolabels='C:\\Users\\e257678\\Desktop\\case20(correction)\\mdl_nolabels.lp'
)

# Console output for quick checks
print("\n✅ Objective Function Value:", pyo.value(mdl.oTotal_Return))
mdl.vX.display()
mdl.vZ.display()



