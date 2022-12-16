// FINAL PROJECT DS210
// Professor Leonidas
// By: Jonathan Hafez

// Imported Crates
use rand::Rng;
use std::fs::File;
use std::io::prelude::*;
use std::io::BufRead;
use std::collections::VecDeque;
use std::sync::Arc;
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use rand::{thread_rng, distributions::Uniform};
use rand::distributions::Distribution;


fn main() {
    println!("\nFind the Average Distance Between Two Vertices in a Graph");
    let mut edg = read_file("fb-pages-company_edges.txt");
    // defining the variable vertices as the maximum value of c and d in the tuples of edg, plus 1, or 0 if edg is empty
    let vertices = edg.iter().flat_map(|&(c, d)| vec![c, d]).max().map(|a| a + 1).unwrap_or(0);

    //create our new graph
    let mut g = Graph::create_undirected(vertices,&edg);

    // PERFORM BFS
    let bfs_on_g = bfs_alg(&g, edg[0].0);

    // Sum the lengths of all the shortest paths found in BFS, 
    // and divide this sum by the total number of pairs of vertices
    // in the graph to find the average distance between two vertices.

    println!("\n- This is the len of our shortest Path found in BFS: --> {:?} \n(we are not printing our entire BFS outuput as it will be a huge output)", bfs_on_g.len());

    // create a variable called maximum that stores the maximum value inside edg
    let maximum = edg.iter().flat_map(|&(c, d)| vec![c, d]).max().map(|a| a + 1).unwrap_or(0);
    // create a variable called minimum that stores the minimum value inside edg
    let minimum = edg.iter().flat_map(|&(c, d)| vec![c, d]).min().map(|a| a + 1).unwrap_or(0);

    let mut total_distance = 0;
    for _ in 0..1000 {
        let initial = rand::thread_rng().gen_range(minimum..maximum);
        let end = rand::thread_rng().gen_range(minimum..maximum);
        let distance = distance_nodes(&g, initial, end);
        total_distance += distance;
    }

    // Calculate the total number of pairs utilized in the calculation
    let total_pairs =1000; // total number of pairs in graph =52311;

    // Calculate the average distance between two vertices
    let average_distance = total_distance as f64/ total_pairs as f64;
    println!("- Total distance in 1000 nodes: {}\n", total_distance);
    println!("- Average distance between two vertices: {}\n(Estimate made based on our selection of 1000s nodes selected randomly from our Graph)\n", average_distance);
    // The average distance is the total distance divided by the total number of pairs,

}

// Computes the distance between two nodes
fn distance_nodes(graph: &Graph, initial: usize, end: usize) -> usize {
    let mut queue = VecDeque::new();
    let mut sum_dist = vec![std::usize::MAX; graph.n];
    let mut past = vec![false; graph.n];

    queue.push_back(initial);
    past[initial] = true;
    sum_dist[initial] = 0;

    while !queue.is_empty() {
        let current = queue.pop_front().unwrap();
        if current == end {
            return sum_dist[end];
        }

        for &neighbor in &graph.outedges[current] {
            if !past[neighbor] {
                queue.push_back(neighbor);
                past[neighbor] = true;

                let distance = sum_dist[current] + 1;
                sum_dist[neighbor] = distance;
            }
        }
    }
    std::usize::MAX
}



// BFS algorithm function
fn bfs_alg(graph: &Graph, node: usize) -> Vec<Vertex> {
    let mut q = VecDeque::new();
    let mut visited = Vec::new();

    q.push_back(node);
    while !q.is_empty() {
        let current_vertex = q.pop_front().unwrap();
        if visited.contains(&current_vertex) {
            continue; }
        visited.push(current_vertex);
        if current_vertex < graph.outedges.len() {
            for &neighbor in &graph.outedges[current_vertex] {
                if !visited.contains(&neighbor) {
                    q.push_back(neighbor);
                }
            }
        }
    }
    println!("Number of Edges Visited: {:?}", visited.len());
    visited
}


// Based on HW10 
fn read_file(_path: &str) -> Vec<(Vertex, Vertex)>{
    let mut result: Vec<(Vertex, Vertex)>  = Vec::new();
    let file = File::open("fb-pages-company_edges.txt").expect("Error opening file");
    let mut buf_reader = std::io::BufReader::new(file).lines();
    buf_reader.next();
    for line in buf_reader {
        let line_str = line.expect("Error reading");
        let v: Vec<&str> = line_str.trim().split(',').collect();
        let a = v[0].parse::<i128>().unwrap();
        let b = v[1].parse::<i128>().unwrap();
        result.push((a as Vertex, b as Vertex));
    }
    return result;
}

/////////////////////////////////////////////////////
// THIS IS IMPORTED FROM LECTURE 28 DS 210
// CREATE A GRAPH TYPE
type Vertex = usize;
type ListOfEdges = Vec<(Vertex,Vertex)>;
type AdjacencyLists = Vec<Vec<Vertex>>;
type Component = usize;

#[derive(Debug)]
struct Graph {
    n: usize, // vertex labels in {0,...,n-1}
    outedges: AdjacencyLists,
}

// reverse direction of edges on a list
fn reverse_edges(list:&ListOfEdges)
        -> ListOfEdges {
    let mut new_list = vec![];
    for (u,v) in list {
        new_list.push((*v,*u));
    }
    new_list
}

// IMPLEMENTATION OF THE GRAPH
impl Graph {
    fn add_directed_edges(&mut self,
                          edges:&ListOfEdges) {
        for (u,v) in edges {
            self.outedges[*u].push(*v);
        }
    }
    fn sort_graph_lists(&mut self) {
        for l in self.outedges.iter_mut() {
            l.sort();
        }
    }
    fn create_directed(n:usize,edges:&ListOfEdges)
                                            -> Graph {
        let mut g = Graph{n,outedges:vec![vec![];n]};
        g.add_directed_edges(edges);
        g.sort_graph_lists();
        g                                        
    }
    
    fn create_undirected(n:usize,edges:&ListOfEdges)
                                            -> Graph {
        let mut g = Self::create_directed(n,edges);
        g.add_directed_edges(&reverse_edges(edges));
        g.sort_graph_lists();
        g                                        
    }
}
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
// impl Di {
//     fn avg_path_length(&self, from: Vertex, to: Vertex) -> f64 {
//         let path_lengths: Vec<f64> = self.get_all_paths(from, to).iter().map(|p| p.len() as f64).collect();
//         let total_length = path_lengths.iter().sum();
//         total_length / path_lengths.len() as f64
//     }

//     fn get_all_paths(&self, from: Vertex, to: Vertex) -> Vec<Vec<Vertex>> {
//         let mut paths = Vec::new();
//         let mut visited = vec![false; self.n];

//         self.get_all_paths_recursive(from, to, &mut visited, &mut Vec::new(), &mut paths);

//         paths
//     }

//     fn get_all_paths_recursive(&self, from: Vertex, to: Vertex, visited: &mut [bool], path: &mut Vec<Vertex>, paths: &mut Vec<Vec<Vertex>>) {
//         visited[from] = true;
//         path.push(from);

//         if from == to {
//             paths.push(path.clone());
//         } else {
//             for &next_node in &self.outedges[from] {
//                 if !visited[next_node] {
//                     self.get_all_paths_recursive(next_node, to, visited, path, paths);
//                 }
//             }
//         }
//     path.pop();
//     visited[from] = false;
// }