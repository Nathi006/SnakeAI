import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;


import za.ac.wits.snake.DevelopmentAgent;
import za.ac.wits.snake.utils.Point;

public class MyAgent extends DevelopmentAgent {

    private static final int UP = 0;
    private static final int DOWN = 1;
    private static final int LEFT = 2;
    private static final int RIGHT = 3;
    
    private int boardWidth;
    private int boardHeight;
    private int numSnakes;
    private int currentTimestep;
    private int lastAppleEatenTimestep;
    private Point lastApplePosition;

    public static void main(String args[]) {
        MyAgent agent = new MyAgent();
        MyAgent.start(agent, args);
    }
   @Override
    public void run() {
        try (BufferedReader br = new BufferedReader(new InputStreamReader(System.in))) {
            // Read initialization
            String initString = br.readLine();
            String[] initParts = initString.split(" ");
            numSnakes = Integer.parseInt(initParts[0]);
            boardWidth = Integer.parseInt(initParts[1]);
            boardHeight = Integer.parseInt(initParts[2]);
            
            currentTimestep = 0;
            lastAppleEatenTimestep = 0;
            lastApplePosition = null;
            
            // Main game loop
            while (true) {
                String line = br.readLine();
                if (line.contains("Game Over")) {
                    break;
                }
                
                // Parse apple position
                String[] appleParts = line.split(" ");
                Point apple = new Point(Integer.parseInt(appleParts[0]),Integer.parseInt(appleParts[1]));
                
                // Track apple changes to calculate its age
                if (lastApplePosition == null || 
                    lastApplePosition.x != apple.x || lastApplePosition.y != apple.y) {
                    lastAppleEatenTimestep = currentTimestep;
                    lastApplePosition = new Point(apple.x, apple.y);
                }
                
                currentTimestep++;
                
                // Read my snake number
                int mySnakeNum = Integer.parseInt(br.readLine());
                
                // Parse all snakes
                Snake[] snakes = new Snake[numSnakes];
                for (int i = 0; i < numSnakes; i++) {
                    String snakeLine = br.readLine();
                    snakes[i] = parseSnake(snakeLine);
                }
                
                Snake mySnake = snakes[mySnakeNum];
                
                // Calculate apple value
                int appleAge = currentTimestep - lastAppleEatenTimestep;
                int appleValue = calculateAppleValue(appleAge);
                
                // Calculate and make move
                if (mySnake.alive) {
                    int move = calculateSurvivalMove(mySnake, snakes, apple, mySnakeNum, appleValue);
                    System.out.println(move);
                } else {
                    System.out.println(UP);
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    private Snake parseSnake(String line) {
        String[] parts = line.split(" ");
        Snake snake = new Snake();
        snake.alive = parts[0].equals("alive");
        
        if (snake.alive) {
            snake.length = Integer.parseInt(parts[1]);
            snake.kills = Integer.parseInt(parts[2]);
            
            snake.body = new ArrayList<>();
            for (int i = 3; i < parts.length; i++) {
                String[] coords = parts[i].split(",");
                snake.body.add(new Point(Integer.parseInt(coords[0]), Integer.parseInt(coords[1])));
            }
        }
        
        return snake;
    }
    
    private int calculateAppleValue(int appleAge) {
        double rawValue = 5.0 - (appleAge * 0.1);
        return (int) Math.ceil(rawValue);
    }
    
    private boolean isAppleSafe(int appleValue) {
        return appleValue > -4;
    }
    
    private boolean shouldSeekApple(int appleValue, int snakeLength, int longestSnakeLength) {
        // Never seek deadly apples
        if (appleValue <= -4) {
            return false;
        }
        
        // Don't seek negative apples if we're already winning
        if (appleValue < 0 && snakeLength >= longestSnakeLength - 3) {
            return false;
        }
        
        // Don't risk shrinking when small
        if (appleValue < 0 && snakeLength <= 8) {
            return false;
        }
        
        // Only seek slightly negative apples if desperate (losing badly)
        if (appleValue < 0 && snakeLength < longestSnakeLength - 10) {
            return appleValue >= -2 && snakeLength > 15;
        }
        
        // Seek positive apples, but be conservative if already long
        if (appleValue > 0) {
            // If we're winning, only seek fresh apples
            if (snakeLength >= longestSnakeLength - 2) {
                return appleValue >= 3;
            }
            return true;
        }
        
        return false;
    }
    
    private int calculateSurvivalMove(Snake mySnake, Snake[] allSnakes, Point apple, int myIndex, int appleValue) {
        Point head = mySnake.body.get(0);
        int currentDir = getCurrentDirection(mySnake);
        
        // Build detailed occupancy grid
        boolean[][] occupied = new boolean[boardWidth][boardHeight];
        boolean[][] dangerZone = new boolean[boardWidth][boardHeight];
        Point myTail = null;
        
        int longestSnakeLength = 0;
        for (Snake snake : allSnakes) {
            if (snake.alive) {
                fillSnakeBody(snake, occupied);
                if (snake.length > longestSnakeLength) {
                    longestSnakeLength = snake.length;
                }
                if (snake == mySnake && snake.body.size() > 0) {
                    myTail = snake.body.get(snake.body.size() - 1);
                }
                // Mark danger zones around other snake heads
                if (snake != mySnake) {
                    markDangerZoneAroundHead(snake, dangerZone);
                }
            }
        }
        
        // Determine strategy based on position
        boolean shouldSeek = shouldSeekApple(appleValue, mySnake.length, longestSnakeLength);
        boolean avoidApple = !isAppleSafe(appleValue);
        
        List<Point> pathToApple = null;
        
        if (shouldSeek) {
            // Always use Dijkstra for survival-focused pathfinding
            pathToApple = dijkstraSurvivalPathfind(head, apple, occupied, dangerZone, myTail, mySnake.length);
        }
        
        if (pathToApple != null && pathToApple.size() > 1) {
            Point nextStep = pathToApple.get(1);
            int move = getDirectionToPoint(head, nextStep);
            
            if (isValidMove(nextStep, occupied) && !dangerZone[nextStep.x][nextStep.y]) {
                // Thorough safety validation
                if (validateSurvivalMove(nextStep, occupied, mySnake.length, pathToApple.size())) {
                    return move;
                }
            }
        }
        
        // Fallback: Find safest survival move
        return findSafestMove(head, currentDir, occupied, dangerZone, apple, mySnake, allSnakes, avoidApple);
    }
    
    private List<Point> dijkstraSurvivalPathfind(Point start, Point goal, boolean[][] occupied, boolean[][] dangerZone, Point myTail, int snakeLength) {
        PriorityQueue<DijkstraNode> openSet = new PriorityQueue<>();
        double[][] distances = new double[boardWidth][boardHeight];
        DijkstraNode[][] nodeMap = new DijkstraNode[boardWidth][boardHeight];
        
        for (int i = 0; i < boardWidth; i++) {
            for (int j = 0; j < boardHeight; j++) {
                distances[i][j] = Double.MAX_VALUE;
            }
        }
        
        DijkstraNode startNode = new DijkstraNode(start, null, 0);
        openSet.add(startNode);
        distances[start.x][start.y] = 0;
        nodeMap[start.x][start.y] = startNode;
        
        int[] dx = {0, 0, -1, 1};
        int[] dy = {-1, 1, 0, 0};
        
        int explored = 0;
        int maxExplore = 2000;
        
        while (!openSet.isEmpty() && explored < maxExplore) {
            DijkstraNode current = openSet.poll();
            Point p = current.point;
            explored++;
            
            if (current.cost > distances[p.x][p.y]) {
                continue;
            }
            
            if (p.x == goal.x && p.y == goal.y) {
                List<Point> path = new ArrayList<>();
                DijkstraNode node = current;
                while (node != null) {
                    path.add(0, node.point);
                    node = node.parent;
                }
                return path;
            }
            
            for (int i = 0; i < 4; i++) {
                int nx = p.x + dx[i];
                int ny = p.y + dy[i];
                
                if (nx < 0 || nx >= boardWidth || ny < 0 || ny >= boardHeight) {
                    continue;
                }
                
                Point next = new Point(nx, ny);
                
                if (occupied[nx][ny]) {
                    if (myTail != null && nx == myTail.x && ny == myTail.y) {
                        // OK
                    } else if (nx == goal.x && ny == goal.y) {
                        // OK
                    } else {
                        continue;
                    }
                }
                
                double edgeCost = calculateSurvivalEdgeCost(p, next, occupied, dangerZone, snakeLength);
                double newCost = current.cost + edgeCost;
                
                if (newCost < distances[nx][ny]) {
                    distances[nx][ny] = newCost;
                    DijkstraNode nextNode = new DijkstraNode(next, current, newCost);
                    nodeMap[nx][ny] = nextNode;
                    openSet.add(nextNode);
                }
            }
        }
        
        return null;
    }
    
    private double calculateSurvivalEdgeCost(Point from, Point to, boolean[][] occupied, boolean[][] dangerZone, int snakeLength) {
        double cost = 1.0;
        
        // Heavy penalty for danger zones (near enemy heads)
        if (dangerZone[to.x][to.y]) {
            cost += 15.0;
        }
        
        // Heavy penalty for edges - survival priority
        int edgeDistance = Math.min(Math.min(to.x, boardWidth - 1 - to.x), 
                                    Math.min(to.y, boardHeight - 1 - to.y));
        if (edgeDistance == 0) {
            cost += 20.0; // On edge
        } else if (edgeDistance == 1) {
            cost += 8.0;  // One square from edge
        } else if (edgeDistance == 2) {
            cost += 3.0;  // Two squares from edge
        }
        
        // Count escape routes - critical for survival
        int[] dx = {0, 0, -1, 1};
        int[] dy = {-1, 1, 0, 0};
        int openNeighbors = 0;
        int safeNeighbors = 0;
        
        for (int i = 0; i < 4; i++) {
            int nx = to.x + dx[i];
            int ny = to.y + dy[i];
            if (nx >= 0 && nx < boardWidth && ny >= 0 && ny < boardHeight) {
                if (!occupied[nx][ny]) {
                    openNeighbors++;
                    if (!dangerZone[nx][ny]) {
                        safeNeighbors++;
                    }
                }
            }
        }
        
        // Heavily penalize low escape options
        if (openNeighbors == 1) {
            cost += 10.0; // Dead end risk
        } else if (openNeighbors == 2) {
            cost += 4.0;  // Limited options
        }
        
        if (safeNeighbors == 0) {
            cost += 8.0; // All neighbors are dangerous
        }
        
        // Penalize narrow corridors for longer snakes
        if (snakeLength > 10) {
            int spaceAround = countImmediateSpace(to, occupied);
            if (spaceAround < 6) {
                cost += (6 - spaceAround) * 2.0;
            }
        }
        
        // Prefer center areas for better maneuverability
        int centerX = boardWidth / 2;
        int centerY = boardHeight / 2;
        double distToCenter = Math.abs(to.x - centerX) + Math.abs(to.y - centerY);
        double maxCenterDist = centerX + centerY;
        cost += (distToCenter / maxCenterDist) * 0.5;
        
        return cost;
    }
    
    private int countImmediateSpace(Point p, boolean[][] occupied) {
        int count = 0;
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                int nx = p.x + dx;
                int ny = p.y + dy;
                if (nx >= 0 && nx < boardWidth && ny >= 0 && ny < boardHeight && !occupied[nx][ny]) {
                    count++;
                }
            }
        }
        return count;
    }
    
    private void markDangerZoneAroundHead(Snake snake, boolean[][] dangerZone) {
        if (snake.body.size() == 0) return;
        
        Point head = snake.body.get(0);
        int[] dx = {0, 0, -1, 1, -1, -1, 1, 1};
        int[] dy = {-1, 1, 0, 0, -1, 1, -1, 1};
        
        // Mark squares around enemy head as dangerous
        for (int i = 0; i < 8; i++) {
            int nx = head.x + dx[i];
            int ny = head.y + dy[i];
            if (nx >= 0 && nx < boardWidth && ny >= 0 && ny < boardHeight) {
                dangerZone[nx][ny] = true;
            }
        }
    }
    
    private boolean validateSurvivalMove(Point nextPos, boolean[][] occupied, int snakeLength, int pathLength) {
        // Deep validation for survival
        boolean[][] tempOccupied = copyGrid(occupied);
        
        // Flood fill to check available space
        int minRequiredSpace = Math.max(snakeLength * 2, 20);
        int availableSpace = countReachableSpacesFast(nextPos, tempOccupied, minRequiredSpace + 10);
        
        if (availableSpace < minRequiredSpace) {
            return false;
        }
        
        // Check if we have multiple escape routes
        int[] dx = {0, 0, -1, 1};
        int[] dy = {-1, 1, 0, 0};
        int escapeRoutes = 0;
        
        for (int i = 0; i < 4; i++) {
            int nx = nextPos.x + dx[i];
            int ny = nextPos.y + dy[i];
            if (nx >= 0 && nx < boardWidth && ny >= 0 && ny < boardHeight && !occupied[nx][ny]) {
                escapeRoutes++;
            }
        }
        
        return escapeRoutes >= 2;
    }
    
    private int findSafestMove(Point head, int currentDir, boolean[][] occupied, boolean[][] dangerZone, Point apple, Snake mySnake, Snake[] allSnakes, boolean avoidApple) {
        int[] moves = {UP, DOWN, LEFT, RIGHT};
        int bestMove = currentDir;
        double bestScore = Double.NEGATIVE_INFINITY;
        
        for (int move : moves) {
            Point newHead = getNewPosition(head, move);
            
            if (!isValidMove(newHead, occupied)) {
                continue;
            }
            
            double score = evaluateSurvivalMove(newHead, apple, mySnake, allSnakes,occupied, dangerZone, avoidApple);
            
            if (score > bestScore) {
                bestScore = score;
                bestMove = move;
            }
        }
        
        return bestMove;
    }
    
    private double evaluateSurvivalMove(Point newHead, Point apple, Snake mySnake, Snake[] allSnakes, boolean[][] occupied, boolean[][] dangerZone, boolean avoidApple) {
        double score = 0;
        
        // Danger zone penalty (highest priority)
        if (dangerZone[newHead.x][newHead.y]) {
            score -= 200;
        }
        
        // Apple distance logic
        double appleDistance = manhattanDistance(newHead, apple);
        if (avoidApple) {
            score += appleDistance * 3;
            if (appleDistance <= 2) {
                score -= 150;
            }
        } else {
            score -= appleDistance * 0.5; // Lower priority than safety
        }
        
        // Space availability (critical)
        int availableSpace = countReachableSpacesFast(newHead, occupied, 30);
        score += availableSpace * 3;
        
        // Escape routes
        int[] dx = {0, 0, -1, 1};
        int[] dy = {-1, 1, 0, 0};
        int openNeighbors = 0;
        int safeNeighbors = 0;
        
        for (int i = 0; i < 4; i++) {
            int nx = newHead.x + dx[i];
            int ny = newHead.y + dy[i];
            if (nx >= 0 && nx < boardWidth && ny >= 0 && ny < boardHeight && !occupied[nx][ny]) {
                openNeighbors++;
                if (!dangerZone[nx][ny]) {
                    safeNeighbors++;
                }
            }
        }
        
        score += openNeighbors * 10;
        score += safeNeighbors * 15;
        
        // Edge avoidance (strong)
        int edgeDistance = Math.min(Math.min(newHead.x, boardWidth - 1 - newHead.x), 
                                    Math.min(newHead.y, boardHeight - 1 - newHead.y));
        if (edgeDistance == 0) {
            score -= 50;
        } else if (edgeDistance == 1) {
            score -= 25;
        } else if (edgeDistance == 2) {
            score -= 10;
        }
        
        // Center preference
        int centerX = boardWidth / 2;
        int centerY = boardHeight / 2;
        double distToCenter = Math.abs(newHead.x - centerX) + Math.abs(newHead.y - centerY);
        score -= distToCenter * 0.3;
        
        // Head collision avoidance
        for (Snake snake : allSnakes) {
            if (snake.alive && snake != mySnake) {
                Point otherHead = snake.body.get(0);
                double dist = manhattanDistance(newHead, otherHead);
                if (dist <= 1) {
                    score -= 100;
                } else if (dist <= 2) {
                    score -= 30;
                }
            }
        }
        
        return score;
    }
    
    private int countReachableSpacesFast(Point start, boolean[][] occupied, int maxDepth) {
        boolean[][] visited = new boolean[boardWidth][boardHeight];
        Queue<Point> queue = new LinkedList<>();
        queue.add(start);
        visited[start.x][start.y] = true;
        int count = 0;
        
        int[] dx = {0, 0, -1, 1};
        int[] dy = {-1, 1, 0, 0};
        
        while (!queue.isEmpty() && count < maxDepth) {
            Point p = queue.poll();
            count++;
            
            for (int i = 0; i < 4; i++) {
                int nx = p.x + dx[i];
                int ny = p.y + dy[i];
                
                if (nx >= 0 && nx < boardWidth && ny >= 0 && ny < boardHeight &&
                    !visited[nx][ny] && !occupied[nx][ny]) {
                    visited[nx][ny] = true;
                    queue.add(new Point(nx, ny));
                }
            }
        }
        
        return count;
    }
    
    private void fillSnakeBody(Snake snake, boolean[][] occupied) {
        for (int i = 0; i < snake.body.size() - 1; i++) {
            Point p1 = snake.body.get(i);
            Point p2 = snake.body.get(i + 1);
            
            int x1 = Math.min(p1.x, p2.x);
            int x2 = Math.max(p1.x, p2.x);
            int y1 = Math.min(p1.y, p2.y);
            int y2 = Math.max(p1.y, p2.y);
            
            for (int x = x1; x <= x2; x++) {
                for (int y = y1; y <= y2; y++) {
                    if (x >= 0 && x < boardWidth && y >= 0 && y < boardHeight) {
                        occupied[x][y] = true;
                    }
                }
            }
        }
    }
    
    private boolean isValidMove(Point p, boolean[][] occupied) {
        return p.x >= 0 && p.x < boardWidth && 
               p.y >= 0 && p.y < boardHeight && 
               !occupied[p.x][p.y];
    }
    
    private Point getNewPosition(Point head, int direction) {
        switch (direction) {
            case UP: return new Point(head.x, head.y - 1);
            case DOWN: return new Point(head.x, head.y + 1);
            case LEFT: return new Point(head.x - 1, head.y);
            case RIGHT: return new Point(head.x + 1, head.y);
            default: return head;
        }
    }
    
    private int getCurrentDirection(Snake snake) {
        if (snake.body.size() < 2) return UP;
        
        Point head = snake.body.get(0);
        Point next = snake.body.get(1);
        
        if (head.y < next.y) return UP;
        if (head.y > next.y) return DOWN;
        if (head.x < next.x) return LEFT;
        if (head.x > next.x) return RIGHT;
        
        return UP;
    }
    
    private int getDirectionToPoint(Point from, Point to) {
        if (to.y < from.y) return UP;
        if (to.y > from.y) return DOWN;
        if (to.x < from.x) return LEFT;
        if (to.x > from.x) return RIGHT;
        return UP;
    }
    
    private boolean[][] copyGrid(boolean[][] grid) {
        boolean[][] copy = new boolean[boardWidth][boardHeight];
        for (int i = 0; i < boardWidth; i++) {
            System.arraycopy(grid[i], 0, copy[i], 0, boardHeight);
        }
        return copy;
    }
    
    private double manhattanDistance(Point p1, Point p2) {
        return Math.abs(p1.x - p2.x) + Math.abs(p1.y - p2.y);
    }
    
    // Helper classes
    private static class Point {
        int x, y;
        Point(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }
    
    private static class Snake {
        boolean alive;
        int length;
        int kills;
        List<Point> body;
    }
    
    private static class DijkstraNode implements Comparable<DijkstraNode> {
        Point point;
        DijkstraNode parent;
        double cost;
        
        DijkstraNode(Point point, DijkstraNode parent, double cost) {
            this.point = point;
            this.parent = parent;
            this.cost = cost;
        }
        
        @Override
        public int compareTo(DijkstraNode other) {
            return Double.compare(this.cost, other.cost);
        }
    }
}
