# Solving TSP Using Integer Programming

from ortools.linear_solver import pywraplp
import numpy as np

# 필요한 파라미터 : s_L, s_R, tau_{ij}, e, N_0, Nplus, C etc.
# 선언해야할 변수 : x,y,p(이는 트럭 루트상에서 j에 오기 전에 i를 방문한 경우), u(이는 트럭 루트에서의 트럭루트의 방문순서), t도 필요함.
# 시간은 어떻게 구할 수 있을까? 
    # ex. 1-> 2 > 3 가는데 2에서 드론이 출발해서 4를 다녀오고 3에서 합체함
        # 2까지는 트럭이 가는대로 가고, 드론이 출발한다고 하면 출발 준비시간 더하고, 트럭의 3도착시간과 드론의 3도착시간을 비교해서 둘중 더 나중의 것을 더하고 이후 랑데뷰시간을 더함.


#  =================================================== Senarios ===============================================

# senario 1 
    # 거리 행렬 정의        # 드론의 배송시간과 트럭의 배송시간은 동일하다고 가정한다.
    # 9-노드(0~8) 거리 행렬 – 대칭, 대각선은 0
    # 이때 0과 8은 starting 및 finishing node
total_tau = [
    [0, 12, 7, 18, 9, 15, 6, 13,0],
    [12, 0, 11, 10, 14, 7, 8, 16,0],
    [7, 11, 0, 17, 5, 9, 10, 12,0],
    [18, 10, 17, 0, 8, 13, 11, 7,0],
    [9, 14, 5, 8, 0, 6, 12, 19,0],
    [15, 7, 9, 13, 6, 0, 16, 10,0],
    [6, 8, 10, 11, 12, 16, 0, 18,0],
    [13, 16, 12, 7, 19, 10, 18, 0,0],
    [0, 12, 7, 18, 9, 15, 6, 13,0]
]
# assumption. Nodes that drone can visit is [2,3,5,7]
dronable_customer = [2, 3, 5, 7]        # C'



# # senario 2 
    # # 거리 행렬 정의        # 드론의 배송시간과 트럭의 배송시간은 동일하다고 가정한다.
    # # 6-노드(0~5) 거리 행렬 – 대칭, 대각선은 0
    # # 이때 0과 8은 starting 및 finishing node
# total_tau = [
#     # 0   1   2   3   4   0
#     [ 0, 12,  7, 15,  9,  0],  # 0: depot
#     [12,  0, 10,  6, 11,  0],  # 1
#     [ 7, 10,  0, 13,  8,  0],  # 2
#     [15,  6, 13,  0, 14,  0],  # 3
#     [ 9, 11,  8, 14,  0,  0],  # 4
#     [ 0, 12,  7, 15,  9,  0]   # 5: finish 
# ]

# dronable_customer = [2, 4]        # C'



# # senario 3
    # # 거리 행렬 정의        # 드론의 배송시간과 트럭의 배송시간은 동일하다고 가정한다.
    # # 4-노드(0~3) 거리 행렬 – 대칭, 대각선은 0
    # # 이때 0과 3은 starting 및 finishing node
# total_tau = [
#     [0,   6,   9,0],   # 0: depot
#     [6,   0,   4,0],   # 1
#     [9,   4,   0,0],
#     [0,   6,   9,0]
# ]
# dronable_customer = [1]



#  =================================================== quantity, set and parameter ===============================================

# 고객 노드 및 전체 노드 개수 정의의
n = len(total_tau)      # number of N_0 : 6
c = len(total_tau) - 2  # number of C : 4


# 집합 정의
N_0 = range(n-1)
Nplus = range(1,n)
C = range(1,n-1)


e = 35      # drone's endurance
sL = 1      # drone launch time
sR = 2      # drone 랑데부 time

BIG_M = 500        # Big-M to tight constraints



# =================================================== Solver ===============================================

# MIP solver 생성
solver = pywraplp.Solver.CreateSolver("CBC")

infinity = solver.infinity()

# =================================================== Variables ===============================================

# 결정 변수: x[i][j] = 1 if truck path from i to j is taken
x = {}
for i in N_0:
    for j in Nplus:
        if i != j:      # 애초에 이런 경로는 없음을 명시시
            x[i, j] = solver.IntVar(0, 1, f'x[{i},{j}]')

# 결정 변수: y[i][j][k] = 1 if drone path from i to j is taken
    # 여기에 P = <i,j,k>는 이미 내포함.
y = {}
for i in N_0:
    for j in range(n):
        for k in Nplus:
            if i != j and k!=i and k!=j and (j in dronable_customer):
                drone_time = total_tau[i][j] + total_tau[j][k]
                if drone_time <= e:
                    y[i, j, k] = solver.IntVar(0, 1, f'y[{i},{j},{k}]')

# decision variable : p[i][j] = 1 if 트럭루트상에서 j전에 i를 방문한다면 1
p = {}
for i in N_0:
    for j in C:
        if i != j:
            p[i, j] = solver.IntVar(0, 1, f'p[{i},{j}]')


# decision variable : t[i]. 이는 트럭 루트 상에서 방문할 시간을 나타냄냄
t = {}
for i in range(n):
    t[i] = solver.NumVar(0, infinity, f't[{i}]')

# decision variable : t[i]. 이는 트럭 루트 상에서 방문할 시간을을 나타냄.
tdot = {}
for i in range(n):
    tdot[i] = solver.NumVar(0, infinity, f'tdot[{i}]')


# Truck 루트의 Subtour 제거를 위한 u 변수 (Miller–Tucker–Zemlin formulation)
u = {}
for i in Nplus:
    u[i] = solver.NumVar(1, c+2, f'u[{i}]')


# =================================================== objective function ===============================================

solver.Minimize(t[n-1])

# =================================================== Constraints ===============================================

# rule : 
# 1. 모든 x에 대해서 -> 그건 for문으로 뺌. 모든 y에 대한 sum -> 그건 solver.Sum안에서 for문 컴프리헨션으로 돌림
# 2. 노드들에 관하여 set에 포함된다 라는 것이 반복적으로 사용되므로 따로 set을 만듦.
# 3. y를 정의할 때 P라는 "y의 경로로 가능한 집합"을 내포하게 정의했기에, 이후에 y가 P에 속해야한다는 조건은 if (i,j,k) in y 로 조건문으로 씀
    # 3-1. 따라서 코드에서는 제약조건이 조금더 간단해진 경향이 있음. 그러나 모든 내용을 담고있음.
# 4. 반면 x에 대한 조건은 y에 비해 까다롭지 않으니 명시하진 않아도 됨. 다만 i != j 정도는 조건으로 추가했음.
    # 4.1 이게 변수를 선언할때 이미 정의된 것들만 사용하면 상관없는데 제약조건에서 선언되지 않은 변수를 사용하려고 하면 그때 오류가 발생함
# 5. 이중 sum은 컴프리헨션의 2중 for문을 활용

# 자주 쓰는 구문
# y의 1중 Sum : solver.Sum(y[i,j,k] for j in range(1,n) if (i, j, k) in y)
# y의 2중 Sum : solver.Sum(y[i,j,k] for j in range(1,n) for i in range(n) if (i, j, k) in y)



for j in C:          # 2
    solver.Add(
        solver.Sum(x[i, j] for i in N_0 if (i, j) in x) +
        solver.Sum(y[i, j, k] for i in N_0 for k in Nplus if (i, j, k) in y) 
        == 1
    )

solver.Add(solver.Sum(x[0, j] for j in Nplus if j != 0) == 1)        # 3
solver.Add(solver.Sum(x[i, n-1] for i in N_0 if i != 0) == 1)        # 4

# Subtour 제거: Miller–Tucker–Zemlin (MTZ)
for i in C:
    for j in Nplus:
        if i != j:
            solver.Add(u[i] - u[j] + 1 <= (c+2)*(1- x[i, j]))         # 5


for j in C:
    solver.Add(solver.Sum(x[i, j] for i in N_0 if i != j) == solver.Sum(x[j, k] for k in Nplus if j != k))      # 6


for i in N_0:
    solver.Add(solver.Sum(y[i, j, k] for j in C for k in Nplus if ((i, j, k) in y)) <= 1)              # 7

for k in Nplus:
    solver.Add(solver.Sum(y[i, j, k] for i in N_0 for j in C if ((i, j, k) in y)) <= 1)              # 8

for i in C:
    for j in C:
        for k in Nplus:
            if (i, j, k) in y:
                solver.Add(2 * y[i,j,k] <= 
                           solver.Sum(x[h,i] for h in N_0 if (h != i)) + 
                           solver.Sum(x[l,k] for l in C if (l != k)))             # 9

for j in C:
    for k in Nplus:
        if (0, j, k) in y:
            solver.Add(y[0,j,k] <=  solver.Sum(x[h,k] for h in N_0 if (h != k)))         # 10


# Subtour 제거: Miller–Tucker–Zemlin (MTZ) for 드론. 원래는 u로부터 x만 순서를 상승시키는 방향이었는데 이제 y도 사실 i랑 k는 트럭이 지나다니는 경로여서 subroute제거가 필요함.
for i in C:        # 11
    for k in Nplus:
        if i != k:
            solver.Add(u[k] - u[i] >= 1 - (c+2)*(1- solver.Sum(y[i,j,k] for j in C if (i, j, k) in y)))  


for i in C:                               # 12
    
    solver.Add(tdot[i] >= t[i] - BIG_M * 
               (1 - 
                solver.Sum(y[i,j,k] for j in C for k in Nplus if (i, j, k) in y)))

for i in C:                               # 13

    solver.Add(tdot[i] <= t[i] + BIG_M * 
               (1 - 
                solver.Sum(y[i,j,k] for j in C for k in Nplus if (i, j, k) in y)))


for k in Nplus:                               # 14

    solver.Add(tdot[k] >= t[k] - BIG_M * 
               (1 - 
                solver.Sum(y[i,j,k] for j in C for i in N_0 if (i, j, k) in y)))


for k in Nplus:                               # 15

    solver.Add(tdot[k] <= t[k] + BIG_M * 
               (1 - 
                solver.Sum(y[i,j,k] for j in C for i in N_0 if (i, j, k) in y)))


for k in Nplus:                          # 16
    for h in N_0:
        if k!=h:
            solver.Add(t[k] >= t[h] + total_tau[h][k] + 
                       sL*solver.Sum(y[k,l,m] for l in C for m in Nplus if (k, l, m) in y)
                         + sR*solver.Sum(y[i,j,k] for j in C for i in N_0 if (i, j, k) in y)
                           - BIG_M * (1 - x[h,k]))



for i in N_0:
    for j in dronable_customer:
        if i!=j:                          # 17
            solver.Add(tdot[j] >= tdot[i] + total_tau[i][j] - 
                       BIG_M * (1 - solver.Sum(y[i, j, k] for k in Nplus if (i, j, k) in y)))

for k in Nplus:
    for j in dronable_customer:
        if k!=j:                          # 18
            solver.Add(tdot[k] >= tdot[j] + total_tau[j][k] + sR - 
                       BIG_M * (1 - solver.Sum(y[i, j, k] for i in N_0 if (i, j, k) in y)))


for i in N_0:                  # 19
    for j in C:
        for k in Nplus:
            if (i, j, k) in y:
                solver.Add(tdot[k] - (tdot[j] - total_tau[i][j]) <= e + BIG_M * (1 - y[i,j,k]))


for i in C:                   # 20
    for j in C:
        if i != j:
            solver.Add(u[i] - u[j] >= 1-(c+2)*p[i,j])

for i in C:                # 21
    for j in C:
        if i != j:
            solver.Add(u[i] - u[j] <= -1+(c+2)*(1-p[i,j]))

for i in C:                # 22
    for j in C:
        if i != j:
            solver.Add(p[i,j] + p[j,i] ==1)


for i in N_0:                  # 23
    for k in Nplus:
        for l in C:
            if i != k and l != i and l != k:
                solver.Add(tdot[l] >= tdot[k] 
                           - BIG_M*(3-solver.Sum(y[i, j, k] for j in C if (i, j, k) in y) 
                                    - solver.Sum(y[l, m, q] for m in C for q in Nplus if (l, m, q) in y)
                                      - p[i,l]))


solver.Add(t[0]==0)         # 24

solver.Add(tdot[0]==0)      # 25

for j in C:        # 26
    solver.Add(p[0,j]==1)



# =================================================== Solve =====================================================

solver.EnableOutput()          # CBC 로그 보기
# solver.SetTimeLimit(10000)     # 10초 제한


# 풀이
status = solver.Solve()


# =================================================== Plotting =====================================================

if status in (pywraplp.Solver.OPTIMAL,
              pywraplp.Solver.FEASIBLE):          # 시간 제한 때는 FEASIBLE
    STATUS_STR = {
    pywraplp.Solver.OPTIMAL   : "OPTIMAL",
    pywraplp.Solver.FEASIBLE  : "FEASIBLE (time-limit)",
    pywraplp.Solver.INFEASIBLE: "INFEASIBLE",
    pywraplp.Solver.UNBOUNDED : "UNBOUNDED",
    pywraplp.Solver.ABNORMAL  : "ABNORMAL",
    pywraplp.Solver.NOT_SOLVED: "NOT_SOLVED",
}

    print(f"\n=== CBC status : {STATUS_STR.get(status, 'UNKNOWN')} ===")
    print(f"목적함수( makespan z ) = {solver.Objective().Value():.2f}\n")

    # ---------- 트럭 경로 ----------
    truck_route = []
    cur = 0
    while True:
        truck_route.append(cur)
        nxt = [j for j in range(n)
               if (cur, j) in x and x[cur, j].solution_value() > 0.5]
        if not nxt:                   # 더 이상 나가는 간선이 없으면 종료
            break
        cur = nxt[0]    
        if cur == 0:                  # 디포 복귀
            truck_route.append(0)
            break
    print("Truck route :", " -> ".join(map(str, truck_route)), "\n")

    # ---------- 드론 임무 ----------
    print("Drone trips (i=launch, j=service, k=rendezvous)")
    for (i, j, k), var in y.items():
        if var.solution_value() > 0.5:
            print(f"  • ({i} → {j} → {k})")

    # ---------- 시간 변수 ----------
    print("\nVisit time of truck   t[i]   |   rendezvous time of drone   tdot[i]")
    for i in range(n):
        print(f"  i={i:2d} :  t = {t[i].solution_value():6.2f}   ",
              f"tdot = {tdot[i].solution_value():6.2f}")

else:
    print("=== 해결 불가(infeasible) 또는 시간 안에 해를 찾지 못함 ===")
