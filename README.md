Python code Solving FSTSP with OR-tools. 

Reference : The flying sidekick traveling salesman problem_Optimization of drone assisted parcel delivery (EN, 2014) By Chase C.Murray, Amanda G. chu

For senario3, it takes about 8 minutes with (Visual Studio Code, i9, Python 3.13.2)

Result of Senatio3 :

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
    
