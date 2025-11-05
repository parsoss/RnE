
import heapq

from typing import Dict, Tuple, List, Optional



class ParkingOptimizer:

    """3D 주차장 ETA 기반 차량 배치 최적화 - A* 알고리즘"""



    def __init__(self):

        # 주차장 구조 (2층 × 4행 × 4열)

        self.FLOORS = 2

        self.ROWS   = 4

        self.COLS   = 4

        # 출입구 위치 (1층 우측 상단)

        self.ENTRANCE = (0, 0, 3)

        # A* 반복 제한 (무한 루프 방지용)

        self.MAX_ITERATIONS = 15000



    def distance_to_entrance(self, p: Tuple[int,int,int]) -> int:

        """

        출입구까지의 실제 이동 거리:

        - 1층(z=0): 단순 맨해튼 거리

        - 2층(z=1): (현재→엘리베이터 수평) + (층간 이동 1) + (엘리베이터→출입구 수평)

        """

        z, y, x = p

        ez, ey, ex = self.ENTRANCE



        # 1층일 때

        if z == ez:

            return abs(y - ey) + abs(x - ex)



        # 2층일 때: 반드시 엘리베이터(0,0) 경유

        to_elev   = abs(y - 0) + abs(x - 0)

        between   = abs(z - ez)         # 1

        from_elev = abs(ey - 0) + abs(ex - 0)

        return to_elev + between + from_elev



    def enhanced_heuristic(self,

                           state: Dict[int,Tuple[int,int,int]],

                           goals: Dict[int,Tuple[int,int,int]]

                          ) -> int:

      

        h_total = 0

        for vid, cur in state.items():

            if vid not in goals:

                continue

            goal = goals[vid]

            z1,y1,x1 = cur

            z2,y2,x2 = goal



            if z1 == z2:

                h_total += abs(y1-y2) + abs(x1-x2)

            else:

                to_elev   = abs(y1-0) + abs(x1-0)

                between   = abs(z1-z2)

                from_elev = abs(y2-0) + abs(x2-0)

                h_total += to_elev + between + from_elev

        return h_total



    def generate_neighbors(

        self,

        state: Dict[int,Tuple[int,int,int]]

    ) -> List[Tuple[Dict[int,Tuple[int,int,int]], Tuple[Tuple[int,int,int],Tuple[int,int,int]]]]:



        occupied = set(state.values())

        nbrs = []



        for vid, (z,y,x) in state.items():

            # 평면 4방향

            for dz,dy,dx in ((0,1,0),(0,-1,0),(0,0,1),(0,0,-1)):

                nz,ny,nx = z+dz, y+dy, x+dx

                if (0 <= nz < self.FLOORS and

                    0 <= ny < self.ROWS   and

                    0 <= nx < self.COLS   and

                    (nz,ny,nx) not in occupied):

                    new = state.copy()

                    new[vid] = (nz,ny,nx)

                    nbrs.append((new, ((nz,ny,nx),(z,y,x))))



            if (y,x) == (0,0):

                for dz in (-1,1):

                    nz = z+dz

                    if 0 <= nz < self.FLOORS and (nz,0,0) not in occupied:

                        new = state.copy()

                        new[vid] = (nz,0,0)

                        nbrs.append((new, ((nz,0,0),(z,y,x))))



        return nbrs



    def generate_goals(

        self,

        eta: Dict[int,int],

        starts: Dict[int,Tuple[int,int,int]]

    ) -> Dict[int,Tuple[int,int,int]]:



        # 1) ETA 순 정렬

        vehicles = sorted(eta.keys(), key=lambda v: eta[v])



        # 2) 후보 위치 수집

        occupied = {pos for pos in starts.values() if pos != self.ENTRANCE}

        cand: List[Tuple[int,Tuple[int,int,int]]] = []

        for z in range(self.FLOORS):

            for y in range(self.ROWS):

                for x in range(self.COLS):

                    p = (z,y,x)

                    if (p == self.ENTRANCE or (y,x)==(0,0) or p in occupied):

                        continue

                    d = self.distance_to_entrance(p)

                    cand.append((d,p))

        cand.sort(key=lambda t: t[0])



        # 3) 매핑

        goals: Dict[int,Tuple[int,int,int]] = {}

        for i, vid in enumerate(vehicles):

            if i < len(cand):

                goals[vid] = cand[i][1]

        return goals



    def a_star_search(

        self,

        starts: Dict[int,Tuple[int,int,int]],

        goals: Dict[int,Tuple[int,int,int]]

    ) -> Optional[List[Tuple[Tuple[int,int,int],Tuple[int,int,int]]]]:



        start_key = tuple(sorted(starts.items()))

        open_set = [(0, 0, start_key, starts)]

        came_from = {}

        g_score = {start_key: 0}

        closed = set()

        iters = 0



        while open_set and iters < self.MAX_ITERATIONS:

            iters += 1

            f, g, sk, st = heapq.heappop(open_set)

            if sk in closed:

                continue

            closed.add(sk)



            if st == goals:

                return self.reconstruct_path(came_from, st)



            for nxt, move in self.generate_neighbors(st):

                nk = tuple(sorted(nxt.items()))

                if nk in closed:

                    continue

                ng = g + 1

                if ng < g_score.get(nk, float('inf')):

                    came_from[nk] = (st, move)

                    g_score[nk] = ng

                    h = self.enhanced_heuristic(nxt, goals)

                    heapq.heappush(open_set, (ng+h, ng, nk, nxt))



        return None



    def reconstruct_path(

        self,

        came_from: Dict[Tuple, Tuple[Dict,Tuple]],

        cur_state: Dict[int,Tuple[int,int,int]]

    ) -> List[Tuple[Tuple[int,int,int],Tuple[int,int,int]]]:

        path = []

        ck = tuple(sorted(cur_state.items()))

        while ck in came_from:

            prev, move = came_from[ck]

            path.append(move)

            ck = tuple(sorted(prev.items()))

        path.reverse()

        return path





def main():

    eta = {1:30, 2:15, 3:45, 4:20}

    starts = {1:(0,1,1), 2:(1,2,2), 3:(0,2,1), 4:(0,0,3)}



    opt = ParkingOptimizer()

    goals = opt.generate_goals(eta, starts)



    print("ETA:", eta)

    print("시작:", starts)

    print("목표:", goals)



    moves = opt.a_star_search(starts, goals)

    if moves:

        print(f"총 이동 횟수: {len(moves)}")

        for i,(new_pos,old_pos) in enumerate(moves,1):

            print(f"{i:2d}) {old_pos} → {new_pos}")

    else:

        print("해를 찾지 못했습니다.")





if __name__ == "__main__":

    main()

