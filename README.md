Python code Solving FSTSP with OR-tools. 

Reference : The flying sidekick traveling salesman problem_Optimization of drone assisted parcel delivery (EN, 2014) By Chase C.Murray, Amanda G. chu

For senario3, it takes about 8 minutes with (Visual Studio Code, i9, Python 3.13.2)


### Result of Senatio3 :

    Result - Optimal solution found

    Objective value:                40.00000000
    Enumerated nodes:               30834
    Total iterations:               1042232
    Time (CPU seconds):             435.12
    Time (Wallclock seconds):       435.12
    
    Total time (CPU seconds):       435.13   (Wallclock seconds):       435.13
        === CBC status : OPTIMAL ===
        목적함수( makespan z ) = 40.00
    
    Truck route : 0 -> 6 -> 1 -> 5 -> 4 -> 2 -> 8
    
    Drone trips (i=launch, j=service, k=rendezvous)
      • (0 → 7 → 5)
      • (5 → 3 → 8)
    
    Visit time of truck   t[i]   |   rendezvous time of drone   tdot[i]
      i= 0 :  t =   0.00    tdot =   0.00
      i= 1 :  t =  15.00    tdot =   0.00
      i= 2 :  t =  38.00    tdot =   0.00
      i= 3 :  t =   0.00    tdot =  38.00
      i= 4 :  t =  33.00    tdot =   0.00
      i= 5 :  t =  25.00    tdot =  25.00
      i= 6 :  t =   6.00    tdot =   0.00
      i= 7 :  t =   0.00    tdot =  13.00
      i= 8 :  t =  40.00    tdot =  40.00

## What you have to pay attention Solving FSTSP with OR-tools?
    
    1. Variable definition : solve.Intvar(a,b,"c") or solve.Numvar(a,b,"c")
        1-1. Dictionaty type
        1-2. a and b is lower bound and upper bound for each.
        1-3. "c" is alias of variable.
        1-4. intvar can only take integer value. Numvar can only take real value.
        1-5. definite variables only exist in domain. It can make constraints easier.
    2. sum terms : solve.Sum() and for comprehension
        2-1. double sum -> using double for comprehension
    3. for all value : use for
    4. definite sets : i.e. C, N_0, Nplus
    5. Separate variables and constrains.
    6. objective function : t[n-1] (arrival time for finishing node)
    7. Way to choose max value : definite a new variable and Add constraints to new variable as lower bound. Then minimize new variable.
    8. " if (i, j, k) in y " it is not valid when y[i,j,k] ==1, but just y[i,j,k] is exist. (Not depend on value of y[i,j,k])
    9. Arrival time in paper is definited as instant of time : Truck and Drone are not stop at any node. With this assumption, the time is called "effective arrival time"
    
    
    
    
    
