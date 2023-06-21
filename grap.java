package 图论;

import java.util.*;

public class Solution {
    //[from节点的值，to节点的值，weight]
    public static Graph createGraph(Integer[][] matrix) {
        Graph graph = new Graph();
        for (int i = 0; i < matrix.length; i++) {
            Integer from = matrix[i][0];
            Integer to = matrix[i][1];
            Integer weight = matrix[i][2];
            if (!graph.nodes.containsKey(from)) {
                graph.nodes.put(from, new Node(from));
            }
            if (!graph.nodes.containsKey(to)) {
                graph.nodes.put(to, new Node(to));
            }
            Node fromNode = graph.nodes.get(from);
            Node toNode = graph.nodes.get(to);
            Edge newEdge = new Edge(weight, fromNode, toNode);
            fromNode.nexts.add(toNode);
            fromNode.out++;
            toNode.in++;
            fromNode.edges.add(newEdge);
            graph.edges.add(newEdge);
        }
        return graph;
    }

    //图的广度优先遍历
    public static void bfs(Node node) {
        if (node == null) {
            return;
        }
        Queue<Node> queue = new LinkedList<>();
        HashSet<Node> set = new HashSet<>();
        queue.add(node);
        set.add(node);
        while (!queue.isEmpty()) {
            Node cur = queue.poll();
            System.out.println(cur.value);
            for (Node next : cur.nexts) {
                if (!set.contains(next)) {
                    set.add(next);
                    queue.add(next);
                }
            }
        }
    }

    //图的深度优先遍历
    public static void dfs(Node node) {
        if (node == null) {
            return;
        }
        Stack<Node> stack = new Stack<>();
        HashSet<Node> set = new HashSet<>();
        stack.add(node);
        set.add(node);
        System.out.println(node.value);
        while (!stack.isEmpty()) {
            Node cur = stack.pop();
            for (Node next : cur.nexts) {
                if (!set.contains(next)) {
                    //将cur压入栈的目的是记录深度遍历的路径
                    stack.push(cur);
                    stack.push(next);
                    set.add(next);
                    System.out.println(next.value);
                    break;
                }
            }
        }
    }

    //图的拓扑排序
    public static List<Node> sortedTopology(Graph graph) {
        //key:某一个node
        //value:剩余的入度
        HashMap<Node, Integer> inMap = new HashMap<>();
        //入度为0的点，才能进入这个队列
        Queue<Node> zeroInQueue = new LinkedList<>();
        for (Node node : graph.nodes.values()) {
            inMap.put(node, node.in);
            if (node.in == 0) {
                zeroInQueue.add(node);
            }
        }
        //将拓扑排序的结果依次加入result
        List<Node> result = new ArrayList<>();
        while (!zeroInQueue.isEmpty()) {
            Node cur = zeroInQueue.poll();
            result.add(cur);
            //调整相邻节点的入度
            for (Node next : cur.nexts) {
                inMap.put(next, inMap.get(next) - 1);
                if (inMap.get(next) == 0) {
                    zeroInQueue.add(next);
                }
            }
        }
        return result;
    }

    //prim算法
    public static Set<Edge> primMST(Graph graph) {
        //解锁的边进入小根堆
        PriorityQueue<Edge> priorityQueue = new PriorityQueue<>(new EdgeComparator());
        //考察过的点都放入HashSet
        HashSet<Node> set = new HashSet<>();
        //依次挑选的边放入result中
        Set<Edge> result = new HashSet<>();
        //随便挑一个点，for循环的目的是处理森林问题
        for (Node node : graph.nodes.values()) {
            if (!set.contains(node)) {
                set.add(node);
                //将该点的邻边都放入小根堆
                for (Edge edge : node.edges) {
                    priorityQueue.add(edge);
                }
                while (!priorityQueue.isEmpty()) {
                    //弹出解锁的边中最小的边
                    Edge edge = priorityQueue.poll();
                    //考察to的点是不是新的点
                    Node toNode = edge.to;
                    //set中不包含to,就是新的点
                    if (!set.contains(toNode)) {
                        set.add(toNode);
                        result.add(edge);
                        for (Edge nextEdge : toNode.edges) {
                            priorityQueue.add(nextEdge);
                        }
                    }
                }
            }
        }
        return result;
    }

    //边比较器
    public static class EdgeComparator implements Comparator<Edge> {
        @Override
        public int compare(Edge o1, Edge o2) {
            return o1.weight - o2.weight;
        }
    }

    //单源最短路径
    public static HashMap<Node, Integer> dijkstra(Node head) {
        //从head出发到所有点的最小距离
        //key:从head出发到达key
        //value:从head出发到达key的最小距离
        //如果在表中，没有T的记录，含义是从head出发到T这个点的距离为正无穷
        HashMap<Node, Integer> distanceMap = new HashMap<>();
        //head到自己的距离为0
        distanceMap.put(head, 0);
        //已经求过距离的点，存在selectNodes里面，再也不碰
        HashSet<Node> selectedNodes = new HashSet<>();
        //getMinDistanceAndUnselectedNode是一个黑盒，很傻的方法，选出最小的Node
        //distanceMap中找到一个最小的记录，处理它，但是这个最小的记录不能是selectedNodes里面的
        //即这个最小的记录不能是已经选过的
        Node minNode = getMinDistanceAndUnselectedNode(distanceMap, selectedNodes);
        while (minNode != null) {
            //第二次循环，下图3，B的distance=3
            int distance = distanceMap.get(minNode);
            //考察所有的边
            for (Edge edge : minNode.edges) {
                //A有3,7,10这三条边，toNode分别是B,C,D
                //第二次循环，B的toNode是A,C，权值分别是3,2，参考下图
                Node toNode = edge.to;
                //such as：B在distanceMap里面没记录，就如上图一样，默认正无穷
                //第二次循环distanceMap中有这条记录，A到A距离为0，所以if内容不执行
                if (!distanceMap.containsKey(toNode)) {
                    //更新距离，A（0）+边的权值为3，所以更新为3如下图
                    distanceMap.put(toNode, distance + edge.weight);
                }
                //toNode的点之前的距离和现在distance+edge的距离有没有更小，选小的
                //不更新，因为A原来的到head距离0，第二次循环distance+权值3+3=6，不更新
                distanceMap.put(edge.to, Math.min(distanceMap.get(toNode), distance + edge.weight));
            }
            //下为锁的代码，minNode被锁进去了，minNode进入到已经被选择的列表中去
            //下下图，A被锁住了
            selectedNodes.add(minNode);
            //在distanceMap里和selecteddNodes中，选一个没选过的最小距离的点选出来
            //第一次循环选出来的就是就是B，参考下下图，因为上面A已经被锁住了
            minNode = getMinDistanceAndUnselectedNode(distanceMap, selectedNodes);
            //然后就是由B开始，参考下图第三张图，继续跑while
            //最终所有节点进selectNodes，getMinDistanceAndUnselectedNode无节点可选
        }
        return distanceMap;
    }

    public static Node getMinDistanceAndUnselectedNode(HashMap<Node, Integer> distanceMap, HashSet<Node> touchedNodes) {
        Node minNode = null;
        //找最短距离
        int minDistance = Integer.MAX_VALUE;
        //
        for (Map.Entry<Node, Integer> entry : distanceMap.entrySet()) {
            Node node = entry.getKey();
            int distance = entry.getValue();
            //点没锁且小于最小距离，更新最小距离与Node
            if (!touchedNodes.contains(node) && distance < minDistance) {
                minNode = node;
                minDistance = distance;
            }
        }
        return minNode;
    }
}

class Graph {
    public HashMap<Integer, Node> nodes; //点集
    public HashSet<Edge> edges; //边集

    public Graph() {
        nodes = new HashMap<>();
        edges = new HashSet<>();
    }
}

class Node {
    public int value;
    public int in; //入度
    public int out; //出度
    public ArrayList<Node> nexts;
    public ArrayList<Edge> edges;

    public Node(int value) {
        this.value = value;
        in = 0;
        out = 0;
        nexts = new ArrayList<>();
        edges = new ArrayList<>();
    }
}

class Edge {
    public int weight;
    public Node from;
    public Node to;

    public Edge(int weight, Node from, Node to) {
        this.weight = weight;
        this.from = from;
        this.to = to;
    }
}
