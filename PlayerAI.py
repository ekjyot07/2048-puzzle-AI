from random import randint
from BaseAI import BaseAI
from ComputerAI import ComputerAI
import math
import time

startTime = 0
timeLimit = 0.2
defaultProbability = 0.9
directionVectors = (UP_VEC, DOWN_VEC, LEFT_VEC, RIGHT_VEC) = (
    (-1, 0), (1, 0), (0, -1), (0, 1))
# (x, y) == (row, col)
maxValWeight = 1
emptinessWeight = 3
smoothWeight = 0.1
monoWeight = 8
disWeight = 10


class PlayerAI(BaseAI):

    def __init__(self):
        self.probability = defaultProbability
        self.possibleNewTiles = [2, 4]
        self.timeLimitReache = False

    def printGrid(self, grid):
        for i in range(grid.size):
            print(grid.map[i])

    def getNewTileValue(self):
        if randint(0, 99) < 100 * self.probability:
            return self.possibleNewTiles[0]
        else:
            return self.possibleNewTiles[1]

    def crossBound(self, pos):
        return pos[0] < 0 or pos[0] > 3 or pos[1] < 0 or pos[1] > 3

    def findFarthestCell(self, pos, dir, grid):
        pos[0] += dir[0]  # row
        pos[1] += dir[1]  # col

        while not self.crossBound(pos) and grid.map[pos[0]][pos[1]] == 0:
            pos[0] += dir[0]
            pos[1] += dir[1]

        return pos

    # measures how smooth the grid is
    def smoothness(self, grid):
        dirs = (directionVectors[1], directionVectors[3])
        smoothness = 0

        for x in range(grid.size):
            for y in range(grid.size):

                if grid.map[x][y]:

                    value = grid.map[x][y]

                    for dir in dirs:
                        adjCell = self.findFarthestCell([x, y], dir, grid)
                        if not self.crossBound(adjCell) and grid.map[adjCell[0]][adjCell[1]] != 0:
                            adjCellValue = grid.map[adjCell[0]][adjCell[1]]
                            smoothness -= abs(value - adjCellValue)

        return smoothness

    # tries to ensure that the values of the tiles are all either increasing or decreasing along both
    # the left/right and up/down directions.
    def monotonicity(self, grid):

        rowScore, colScore = 0, 0

        #dirs = (down, right)
        dirs = (directionVectors[1], directionVectors[3])

        # left-right
        for x in range(grid.size):

            currentIndex = 0
            nextIndex = 0

            while not self.crossBound([x, nextIndex+1]):

                [x, nextIndex] = self.findFarthestCell(
                    [x, nextIndex], dirs[1], grid)

                if nextIndex >= 4:
                    nextIndex -= 1

                currentValue = math.log2(
                    grid.map[x][currentIndex]) if grid.map[x][currentIndex] else 0
                nextValue = math.log2(
                    grid.map[x][nextIndex]) if grid.map[x][nextIndex] else 0

                rowScore += currentValue - nextValue

                currentIndex = nextIndex

        # up-down
        for y in range(grid.size):

            currentIndex = 0
            nextIndex = 0

            while not self.crossBound([nextIndex+1, y]):

                [nextIndex, y] = self.findFarthestCell(
                    [nextIndex, y], dirs[0], grid)

                if nextIndex >= 4:
                    nextIndex -= 1

                currentValue = math.log2(
                    grid.map[currentIndex][y]) if grid.map[currentIndex][y] else 0
                nextValue = math.log2(
                    grid.map[nextIndex][y]) if grid.map[nextIndex][y] else 0

                colScore += currentValue - nextValue

                currentIndex = nextIndex

        return abs(rowScore) + abs(colScore)

    def getTotalSum(self, grid):
        sum = 0
        for i in range(grid.size):
            for j in range(grid.size):
                sum += grid.getCellValue([i, j])

        return math.log2(sum)

    # distance of the biggest tile from the top-left corner
    def distanceFromCorner(self, grid, maxTile):
        distance = None

        for x in range(grid.size):

            for y in range(grid.size):

                if maxTile == grid.map[x][y]:
                    distance = -((abs(x - 0) + abs(y - 0))
                                 * math.log2(maxTile))
                    break

            if distance:
                break

        return distance

    # calculate utility function for a terminal node -- timecutoff or depth == 0

    def evaluate(self, grid):
        return maxValWeight*len(grid.getAvailableCells())*math.log2(grid.getMaxTile()) + emptinessWeight*len(grid.getAvailableCells()) + smoothWeight*self.smoothness(grid) + monoWeight*self.monotonicity(grid) + disWeight*self.distanceFromCorner(grid, grid.getMaxTile())

    # minmax search with alpha beta pruning

    def search(self, grid, alpha, beta, depth, endTime, player):

        if time.perf_counter() > endTime:
            self.timeLimitReached = True
            return self.evaluate(grid), None

        if depth == 0:
            return self.evaluate(grid), None

        # player's turn
        if player:

            bestScore = alpha
            bestMove = None
            moves = grid.getAvailableMoves()

            if len(moves) < 1:
                return self.evaluate(grid), None

            for move in moves:

                childGrid = grid.clone()
                childGrid.move(move)

                score, mv = self.search(
                    childGrid, alpha, beta, depth - 1, endTime, False)

                if score > bestScore:
                    bestScore = score
                    bestMove = move

                if bestScore >= beta:
                    break

                if bestScore > alpha:
                    alpha = bestScore

            return bestScore, bestMove

        # computer's turn
        else:
            bestScore = beta
            bestMove = None

            cells = grid.getAvailableCells()

            if len(cells) < 1:
                return self.evaluate(grid), None

            for cell in cells:

                childGrid = grid.clone()
                childGrid.setCellValue(cell, self.getNewTileValue())

                score, mv = self.search(
                    childGrid, alpha, beta, depth - 1, endTime, True)

                if score < bestScore:
                    bestScore = score
                    bestMove = None

                if bestScore <= alpha:
                    break

                if bestScore < beta:
                    beta = bestScore

            return bestScore, None

    # returns the best move, uses iterative deepeing
    def getMove(self, grid):

        global startTime
        bestScore = -math.inf
        depth = 1
        score = 0
        self.timeLimitReached = False

        startTime = time.perf_counter()
        endTime = startTime + timeLimit

        while True:

            score, move = self.search(
                grid, -math.inf, math.inf, depth, endTime, True)

            if self.timeLimitReached:
                break

            if score > bestScore:
                bestMove = move
                bestScore = score

            depth += 1

        # print('bestScore: ' + str(bestScore))

        # nextGrid = grid.clone()
        # nextGrid.move(bestMove)

        # print('MaxVal: ' + str(maxValWeight*math.log2(nextGrid.getMaxTile())) + ', avalilableCells: ' +
        #  str(emptinessWeight*len(nextGrid.getAvailableCells())) + ', smoothness: ' + str(smoothWeight*self.smoothness(nextGrid))
        #  + ', monotonocity: ' + str(monoWeight*self.monotonicity(nextGrid)) + ', dis: ' + str(disWeight*self.distanceFromCorner(nextGrid, nextGrid.getMaxTile())))

        return bestMove
