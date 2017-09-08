using MoreLinq;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;

namespace SpeedDating
{
    public static class SpeedDating
    {
        #region Graph definitions

        public class Vertex
        {
            public int Index { get; }

            public string Name { get; }

            public IList<Edge> Edges { get; }

            public IList<Vertex> Neighbours => GetNeighbours();

            public Vertex(int index, string name)
            {
                Index = index;
                Name = name;
                Edges = new List<Edge>();
            }

            private List<Vertex> GetNeighbours()
            {
                var neighbours = new List<Vertex>();
                foreach (var edge in Edges)
                {
                    neighbours.Add(edge.Index1 == Index ? edge.Vertex2 : edge.Vertex1);
                }
                return neighbours;
            }

            public override string ToString()
            {
                return Name;
            }

            public override bool Equals(object obj)
            {
                var vertex = obj as Vertex;
                if (vertex != null) return Index == vertex.Index;
                return base.Equals(obj);
            }

            public override int GetHashCode()
            {
                return Index.GetHashCode();
            }
        }

        public class Edge
        {
            public Tuple<Vertex, Vertex> Vertices { get; }

            public int Value { get; }

            public Vertex Vertex1 => Vertices.Item1;

            public Vertex Vertex2 => Vertices.Item2;

            public int Index1 => Vertex1.Index;

            public int Index2 => Vertex2.Index;

            public Edge(Vertex vertex1, Vertex vertex2, int value)
            {
                // Always put the lower index first
                if (vertex1.Index <= vertex2.Index)
                {
                    Vertices = new Tuple<Vertex, Vertex>(vertex1, vertex2);
                }
                else
                {
                    Vertices = new Tuple<Vertex, Vertex>(vertex2, vertex1);
                }
                Value = value;
            }

            public override string ToString()
            {
                return $"{Vertex1.Name} <--> {Vertex2.Name}";
            }

            public override bool Equals(object obj)
            {
                var edge = obj as Edge;
                if (edge != null) return Index1 == edge.Index1 && Index2 == edge.Index2;
                return base.Equals(obj);
            }

            public override int GetHashCode()
            {
                return (Index1 + Index2).GetHashCode();
            }
        }

        public class Graph
        {
            private readonly IList<Vertex> _vertices;
            private readonly IList<Edge> _edges;

            public IReadOnlyList<Vertex> Vertices { get; }

            public IReadOnlyList<Edge> Edges { get; }

            private readonly Dictionary<Tuple<int, int>, Edge> _edgeDictionary;

            public Graph()
            {
                _vertices = new List<Vertex>();

                Vertices = new ReadOnlyCollection<Vertex>(_vertices);

                _edges = new List<Edge>();
                Edges = new ReadOnlyCollection<Edge>(_edges);

                _edgeDictionary = new Dictionary<Tuple<int, int>, Edge>();

                foreach (var edge in _edges)
                {
                    _edgeDictionary.Add(new Tuple<int, int>(edge.Index1, edge.Index2), edge);
                }
            }

            public Edge GetEdge(int index1, int index2)
            {
                if (_edgeDictionary.TryGetValue(new Tuple<int, int>(index1, index2), out var edge))
                {
                    return edge;
                }
                if (_edgeDictionary.TryGetValue(new Tuple<int, int>(index2, index1), out edge))
                {
                    return edge;
                }
                throw new EdgeNotFoundException(index1, index2);
            }

            public Vertex GetVertex(int index)
            {
                var vertex = _vertices.FirstOrDefault(v => v.Index == index);
                if (vertex == null) throw new VertexNotFoundException(index);
                return vertex;
            }

            public void RemoveEdge(int index1, int index2)
            {
                var edgeToRemove = GetEdge(index1, index2);
                _edges.Remove(edgeToRemove);
                edgeToRemove.Vertex1.Edges.Remove(edgeToRemove);
                edgeToRemove.Vertex2.Edges.Remove(edgeToRemove);
                _edgeDictionary.Remove(new Tuple<int, int>(index1, index2));
            }

            public void RemoveEdge(Edge edge)
            {
                RemoveEdge(edge.Index1, edge.Index2);
            }

            public void AddEdge(int index1, int index2, int value)
            {
                var vertex1 = Vertices.FirstOrDefault(v => v.Index == index1);
                if (vertex1 == null) throw new VertexNotFoundException(index1);

                var vertex2 = Vertices.FirstOrDefault(v => v.Index == index2);
                if (vertex2 == null) throw new VertexNotFoundException(index2);

                var newEdge = new Edge(vertex1, vertex2, value);
                _edges.Add(newEdge);
                vertex1.Edges.Add(newEdge);
                vertex2.Edges.Add(newEdge);
                _edgeDictionary.Add(new Tuple<int, int>(index1, index2), newEdge);
            }

            public void AddEdge(Edge edge)
            {
                AddEdge(edge.Index1, edge.Index2, edge.Value);
            }

            public void RemoveVertex(Vertex vertex)
            {
                _vertices.Remove(vertex);
                var edgesToRemove = _edges.Where(edge => Equals(edge.Vertex1, vertex) || Equals(edge.Vertex2, vertex)).ToList();
                foreach (var edge in edgesToRemove)
                {
                    RemoveEdge(edge);
                }
            }

            public void RemoveVertex(int index)
            {
                RemoveVertex(GetVertex(index));
            }

            public void AddVertex(Vertex vertex)
            {
                var newVertex = new Vertex(vertex.Index, vertex.Name);
                _vertices.Add(newVertex);
            }

            public void AddVertex(int index, string name)
            {
                AddVertex(new Vertex(index, name));
            }


            /// <summary>
            /// Get the distance between two nodes using Dijkstra's algorithm. Returns -1 if there is no path.
            /// Out parameter yields the minimal path found.
            /// </summary>
            public int GetDistance(Vertex startingVertex, Vertex targetVertex, out Graph path)
            {
                // Setup
                var tentativeDistances = new Dictionary<Vertex, int>();
                var previousNodes = new Dictionary<Vertex, Vertex>();
                var unvisitedNodes = new List<Vertex>();

                foreach (var vertex in Vertices)
                {
                    tentativeDistances.Add(vertex, int.MaxValue);
                    previousNodes.Add(vertex, null);
                    unvisitedNodes.Add(vertex);
                }

                tentativeDistances[startingVertex] = 0;

                // Do Dijkstra's algorithm
                while (unvisitedNodes.Contains(targetVertex) && unvisitedNodes.Any(v => tentativeDistances[v] != int.MaxValue))
                {
                    var currentVertex = unvisitedNodes.MinBy(v => tentativeDistances[v]);
                    foreach (var neighbour in currentVertex.Neighbours)
                    {
                        var currentVertexDistance = tentativeDistances[currentVertex];
                        var neighbourTentativeDistance = tentativeDistances[neighbour];
                        if (neighbourTentativeDistance > currentVertexDistance + 1)
                        {
                            tentativeDistances[neighbour] = currentVertexDistance + 1;
                            previousNodes[neighbour] = currentVertex;
                        }
                    }
                    unvisitedNodes.Remove(currentVertex);
                }

                if (tentativeDistances[targetVertex] == int.MaxValue)
                {
                    path = null;
                    return int.MaxValue;
                }

                // Also return the shortest path found
                path = new Graph();
                var mostRecentVertex = targetVertex;
                path.AddVertex(mostRecentVertex);

                while (previousNodes[mostRecentVertex] != null)
                {
                    var previousVertex = previousNodes[mostRecentVertex];
                    path.AddVertex(previousVertex);
                    path.AddEdge(GetEdge(mostRecentVertex.Index, previousVertex.Index));
                    mostRecentVertex = previousVertex;
                }

                return tentativeDistances[targetVertex];
            }

            public int GetDistance(Vertex vertex1, Vertex vertex2)
            {
                return GetDistance(vertex1, vertex2, out var dummy);
            }
            public int GetDistance(int index1, int index2)
            {
                return GetDistance(GetVertex(index1), GetVertex(index2), out var dummy);
            }

            public int GetDistance(int index1, int index2, out Graph path)
            {
                return GetDistance(GetVertex(index1), GetVertex(index2), out path);
            }

            /// <summary>
            /// Contracts the given vertices into a single vertex.
            /// </summary>
            public void ContractVertices(IList<int> indices, string contractedVertexName)
            {
                var vertices = Vertices.Where(v => indices.Contains(v.Index)).ToList();
                var edgesToRemove = new List<Edge>();
                var index = Vertices.Max(v => v.Index) + 1;
                var contractedVertex = new Vertex(index, contractedVertexName);
                AddVertex(contractedVertex);

                foreach (var vertex in vertices)
                {
                    foreach (var edge in vertex.Edges)
                    {
                        Edge edgeStickingOut = null;
                        if (!vertices.Contains(edge.Vertex1))
                        {
                            edgeStickingOut = new Edge(edge.Vertex1, contractedVertex, edge.Value);
                        }
                        else if (!vertices.Contains(edge.Vertex2))
                        {
                            edgeStickingOut = new Edge(contractedVertex, edge.Vertex2, edge.Value);
                        }
                        if (edgeStickingOut != null && Edges.All(e => !Equals(e, edgeStickingOut)))
                        {
                            AddEdge(edgeStickingOut);
                        }
                        if (!edgesToRemove.Contains(edge))
                        {
                            edgesToRemove.Add(edge);
                            
                        }
                    }
                }
                foreach (var edge in edgesToRemove) RemoveEdge(edge);
                foreach (var vertex in vertices) RemoveVertex(vertex);
            }

            public void ContractVertices(IList<Vertex> vertices, string contractedVertexName)
            {
                var indices = vertices.Select(v => v.Index).ToList();
                ContractVertices(indices, contractedVertexName);
            }

            public override string ToString()
            {
                var graphString = string.Empty;
                var sortedVertices = Vertices.OrderBy(v => v.Index);
                foreach (var vertex in sortedVertices)
                {
                    foreach (var edge in vertex.Edges)
                    {
                        // Only print each edge one time, not once for each vertex!
                        if (Equals(vertex, edge.Vertex1))
                        {
                            graphString += $"{vertex.Name} paired with {edge.Vertex2.Name}\n";
                        }
                    }
                }
                return graphString;
            }
        }

        // Exceptions

        public class EdgeNotFoundException : Exception
        {
            public EdgeNotFoundException(int index1, int index2) : base(
                $"Edge ({index1}, {index2}) not found in graph")
            {
            }
        }

        public class VertexNotFoundException : Exception
        {
            public VertexNotFoundException(int index) : base(
                $"Vertex {index} not found in graph")
            {
            }
        }

        #endregion

        #region Blossom algorithm

        public static Graph FindOptimalMatching(Graph graph, Graph matching)
        {
            var augmentingPath = FindAugmentingPath(graph, matching);
            if (augmentingPath != null)
            {
                var augmentedMatching = AugmentMatching(matching, augmentingPath);
                return FindOptimalMatching(graph, augmentedMatching);
            }
            return matching;
        }

        private static Graph FindAugmentingPath(Graph graph, Graph matching)
        {
            var forest = new Graph();
            var markedVertices = new List<Vertex>();
            var pathRoots = new List<Vertex>();
            var markedEdges = new List<Edge>();

            foreach (var vertex in matching.Vertices)
            {
                if (vertex.Edges.Count == 0) // Vertex is exposed
                {
                    var newVertex = new Vertex(vertex.Index, vertex.Name);
                    forest.AddVertex(newVertex);
                    pathRoots.Add(newVertex);
                }
            }

            while (true)
            {
                var vertexToCheck = forest.Vertices.FirstOrDefault(v => !markedVertices.Contains(v)
                    && pathRoots.Any(r => forest.GetDistance(v, r) % 2 == 0));
                if (vertexToCheck == null) return null;

                while (true)
                {
                    var edgeToCheck = graph.Edges.FirstOrDefault(e => !markedEdges.Contains(e) &&
                        (Equals(e.Vertex1, vertexToCheck) || Equals(e.Vertex2, vertexToCheck)));
                    if (edgeToCheck == null) break;

                    var otherVertex = Equals(edgeToCheck.Vertex1, vertexToCheck) ? edgeToCheck.Vertex2 : edgeToCheck.Vertex1;

                    if (!forest.Vertices.Contains(otherVertex))
                    {
                        // otherVertex is matched, so add edgeToCheck and otherVertex's matched edge to the path.
                        forest.AddVertex(otherVertex);
                        var matchOfOtherVertex = matching.GetVertex(otherVertex.Index).Neighbours[0];
                        forest.AddVertex(matchOfOtherVertex);
                        forest.AddEdge(edgeToCheck.Index1, edgeToCheck.Index2,edgeToCheck.Value);
                        forest.AddEdge(matching.Edges.First(e => Equals(e.Vertex1, otherVertex) || Equals(e.Vertex2, otherVertex)));
                    }
                    else
                    {
                        if (pathRoots.Any(r => forest.GetDistance(otherVertex, r) < int.MaxValue && forest.GetDistance(otherVertex, r) % 2 == 1))
                        {
                            // Do nothing
                        }
                        else
                        {
                            var rootOfVertexToCheck = pathRoots.First(r => forest.GetDistance(vertexToCheck, r) < int.MaxValue);
                            var rootOfOtherVertex = pathRoots.First(r => forest.GetDistance(otherVertex, r) < int.MaxValue);
                            if (!Equals(rootOfVertexToCheck, rootOfOtherVertex))
                            {
                                // Report that an augmenting path has been found!
                                forest.AddEdge(edgeToCheck);
                                forest.GetDistance(rootOfVertexToCheck, rootOfOtherVertex, out var augmentingPath);
                                return augmentingPath;
                            }
                            // Otherwise, contract a blossom in G and look for a path in the contracted graph instead
                            var contractedGraph = new Graph();
                            foreach (var vertex in graph.Vertices)
                            {
                                contractedGraph.AddVertex(vertex);
                            }
                            foreach (var edge in graph.Edges)
                            {
                                contractedGraph.AddEdge(edge);
                            }

                            var contractedMatching = new Graph();
                            foreach (var vertex in matching.Vertices)
                            {
                                contractedMatching.AddVertex(vertex);
                            }
                            foreach (var edge in matching.Edges)
                            {
                                contractedMatching.AddEdge(edge);
                            }

                            forest.GetDistance(vertexToCheck, otherVertex, out var blossom);
                            blossom.AddEdge(edgeToCheck);
                            contractedGraph.ContractVertices((IList<Vertex>) blossom.Vertices, "ContractionPoint");
                            contractedMatching.ContractVertices((IList<Vertex>) blossom.Vertices, "ContractionPoint");
                            var contractionPointIndex = contractedGraph.Vertices.Max(v => v.Index);

                            var contractedPath = FindAugmentingPath(contractedGraph, contractedMatching);
                            if (contractedPath == null) return null;

                            // Lift the contraction to a proper augmented path
                            var augmentedPath = new Graph();
                            foreach (var vertex in contractedPath.Vertices)
                            {
                                augmentedPath.AddVertex(vertex);
                            }
                            foreach (var edge in contractedPath.Edges)
                            {
                                augmentedPath.AddEdge(edge);
                            }

                            var contractionPoint = contractedPath.Vertices.FirstOrDefault(v => v.Index == contractionPointIndex);
                            if (contractionPoint == null)
                            {
                                // The path doesn't go through the blossom anyway, no need to lift it
                                return augmentedPath;
                            }
                            augmentedPath.RemoveVertex(contractionPoint);

                            if (contractionPoint.Neighbours.Count == 2)
                            {
                                // The path goes through the blossom. Build a completing path segment
                                var startPointOutsideTheFlower = graph.GetVertex(contractionPoint.Neighbours[0].Index);
                                var endPointOutsideTheFlower = graph.GetVertex(contractionPoint.Neighbours[1].Index);
                                var startPointInsideTheFlower =
                                    blossom.Vertices.First(v => startPointOutsideTheFlower.Neighbours.Contains(v));
                                var endPointInsideTheFlower =
                                    blossom.Vertices.First(v => endPointOutsideTheFlower.Neighbours.Contains(v));

                                var pathLength = blossom.GetDistance(startPointInsideTheFlower, endPointInsideTheFlower,
                                    out var pathInsideTheFlower);
                                if (pathLength % 2 == 0)
                                {
                                    // Add the path segment to the augmented path
                                    foreach (var vertex in pathInsideTheFlower.Vertices)
                                    {
                                        augmentedPath.AddVertex(vertex);
                                    }
                                    foreach (var edge in pathInsideTheFlower.Edges)
                                    {
                                        augmentedPath.AddEdge(edge);
                                    }
                                    augmentedPath.AddEdge(graph.GetEdge(startPointOutsideTheFlower.Index, startPointInsideTheFlower.Index));
                                    augmentedPath.AddEdge(graph.GetEdge(endPointInsideTheFlower.Index, endPointOutsideTheFlower.Index));
                                }
                                else
                                {
                                    // Add the *other* path segment to the augmented path
                                    var verticesToAdd =
                                        blossom.Vertices.Where(v => !pathInsideTheFlower.Vertices.Contains(v)).ToList();
                                    var edgesToAdd =
                                        blossom.Edges.Where(v => !pathInsideTheFlower.Edges.Contains(v)).ToList();

                                    augmentedPath.AddVertex(startPointInsideTheFlower);
                                    augmentedPath.AddVertex(endPointInsideTheFlower);
                                    foreach (var vertex in verticesToAdd)
                                    {
                                        augmentedPath.AddVertex(vertex);
                                    }
                                    foreach (var edge in edgesToAdd)
                                    {
                                        augmentedPath.AddEdge(edge);
                                    }
                                    augmentedPath.AddEdge(graph.GetEdge(startPointOutsideTheFlower.Index, startPointInsideTheFlower.Index));
                                    augmentedPath.AddEdge(graph.GetEdge(endPointInsideTheFlower.Index, endPointOutsideTheFlower.Index));
                                }


                            }
                            else
                            {
                                // Flower contraction point is an endpoint
                                var startPointOutsideTheFlower = graph.GetVertex(contractionPoint.Neighbours[0].Index);
                                var startPointInsideTheFlower =
                                    blossom.Vertices.First(v => startPointOutsideTheFlower.Neighbours.Contains(v));
                                var endPointInsideTheFlower =
                                    blossom.Vertices.First(v => matching.GetVertex(v.Index).Neighbours.Count == 0);

                                var pathLength = blossom.GetDistance(startPointInsideTheFlower, endPointInsideTheFlower,
                                    out var pathInsideTheFlower);
                                if (pathLength % 2 == 0)
                                {
                                    // Add the path segment to the augmented path
                                    foreach (var vertex in pathInsideTheFlower.Vertices)
                                    {
                                        augmentedPath.AddVertex(vertex);
                                    }
                                    foreach (var edge in pathInsideTheFlower.Edges)
                                    {
                                        augmentedPath.AddEdge(edge);
                                    }
                                    augmentedPath.AddEdge(graph.GetEdge(startPointOutsideTheFlower.Index, startPointInsideTheFlower.Index));
                                }
                                else
                                {
                                    // Add the *other* path segment to the augmented path
                                    var verticesToAdd =
                                        blossom.Vertices.Where(v => !pathInsideTheFlower.Vertices.Contains(v)).ToList();
                                    var edgesToAdd =
                                        blossom.Edges.Where(v => !pathInsideTheFlower.Edges.Contains(v)).ToList();

                                    augmentedPath.AddVertex(startPointInsideTheFlower);
                                    augmentedPath.AddVertex(endPointInsideTheFlower);
                                    foreach (var vertex in verticesToAdd)
                                    {
                                        augmentedPath.AddVertex(vertex);
                                    }
                                    foreach (var edge in edgesToAdd)
                                    {
                                        augmentedPath.AddEdge(edge);
                                    }
                                    augmentedPath.AddEdge(graph.GetEdge(startPointOutsideTheFlower.Index, startPointInsideTheFlower.Index));
                                }
                            }
                            return augmentedPath;
                        }
                    }
                    markedEdges.Add(edgeToCheck);
                }
                markedVertices.Add(vertexToCheck);
            }
        }

        private static Graph AugmentMatching(Graph matching, Graph augmentingPath)
        {
            var augmentedMatching = new Graph();
            foreach (var vertex in matching.Vertices)
            {
                augmentedMatching.AddVertex(vertex);
            }
            foreach (var edge in matching.Edges)
            {
                if (!augmentingPath.Edges.Contains(edge))
                {
                    augmentedMatching.AddEdge(edge);
                }
            }
            foreach (var edge in augmentingPath.Edges)
            {
                if (!matching.Edges.Contains(edge))
                {
                    augmentedMatching.AddEdge(edge);
                }
            }
            return augmentedMatching;
        }

        #endregion

        public static Graph CreateCompleteUnweightedGraph(IList<string> vertexNames)
        {
            var graph = CreateEmptyInitialMatching(vertexNames);
            for (int i = 1; i <= vertexNames.Count; i++)
            {
                for (int j = i + 1; j <= vertexNames.Count; j++)
                {
                    graph.AddEdge(i,j,1);
                }
            }
            return graph;
        }

        public static Graph CreateEmptyInitialMatching(IList<string> vertexNames)
        {
            var graph = new Graph();
            for (int i = 1; i <= vertexNames.Count; i++)
            {
                graph.AddVertex(i, vertexNames[i - 1]);
            }
            return graph;
        }

        public static void RemoveSameProjectEdgesFromEntireGraph(Graph graph)
        {
            var edgesToRemove = graph.Edges.Where(edge => GetProjectFromVertexName(edge.Vertex1.Name) == GetProjectFromVertexName(edge.Vertex2.Name)).ToList();
            foreach (var edge in edgesToRemove)
            {
                graph.RemoveEdge(edge);
            }
        }

        public static void RemoveProjectEdgesFromVertex(Graph graph, Vertex vertex, string projectName)
        {
            var edgesToRemove = new List<Edge>();
            foreach (var edge in vertex.Edges)
            {
                var otherVertex = Equals(edge.Vertex1, vertex) ? edge.Vertex2 : edge.Vertex1;
                if (GetProjectFromVertexName(otherVertex.Name) == projectName)
                {
                    edgesToRemove.Add(edge);
                }
            }
            foreach (var edge in edgesToRemove)
            {
                graph.RemoveEdge(edge);
            }
        }

        private static string GetProjectFromVertexName(string vertexName)
        {
            return vertexName.Split('(', ')')[1];
        }

        private static void WriteLine(string text, TextWriter textWriter)
        {
            Console.WriteLine(text);
            textWriter.WriteLine(text);
        }


        public static void Main(string[] args)
        {
            using (var streamWriter = new StreamWriter("SpeedDating.txt"))
            {
                var numberOfMatchings = 5;

                var vertexNames = new List<string>
                {
                    "Dilbert Dilbertsson (Skatteverket)",
                    "Mister Basement (Källarprojektet)",
                    "Iron Man (Avengers)",
                    "Captain America (Avengers)",
                    "Hawkeye (Hemmafru)",
                    "Black Widow (Avengers)",
                    "Thor (Avengers)",
                    "Dr. Horrible (Evil League of Evil)",
                    "Andreas (Attentec)",
                    "Chell (Aperture)",
                    "Ratman (Aperture)",
                    "Scatman (Scoobidy Bap Babbidyboo-projektet)",
                    "Euler (Project Euler)",
                    "Gauss (Project Euler)",
                    "Grand Moff Tarkin (Death Star)",
                    "Mike Wazowski (Monsters Inc.)",
                    "Vin Diesel (Trafikverket)",
                };

                if (vertexNames.Count % 2 != 0)
                {
                    vertexNames.Add("The Equalizer (Det Måste Vara Jämnt-projektet)");
                }

                // Setup graph
                var graph = CreateCompleteUnweightedGraph(vertexNames);
                var emptyMatching = CreateEmptyInitialMatching(vertexNames);
                RemoveSameProjectEdgesFromEntireGraph(graph);

                // Avoid matching Dr. Horrible with Avengers, since that would be awkward
                RemoveProjectEdgesFromVertex(graph, graph.Vertices.First(v => v.Name == "Dr. Horrible (Evil League of Evil)"), "Avengers");

                WriteLine($"Matching the following {vertexNames.Count} participants:", streamWriter);
                foreach (var name in vertexNames)
                {
                    WriteLine(name, streamWriter);
                }

                // Find matchings
                var listOfMatchings = new List<Graph>();
                for (int i = 0; i < numberOfMatchings; i++)
                {
                    WriteLine($"\n--ROUND {i + 1}--", streamWriter);
                    var optimalMatching = FindOptimalMatching(graph, emptyMatching);

                    WriteLine(optimalMatching.ToString(), streamWriter);
                    listOfMatchings.Add(optimalMatching);

                    if (optimalMatching.Edges.Count * 2 != vertexNames.Count)
                    {
                        WriteLine("Failed to match:", streamWriter);
                        foreach (var vertex in optimalMatching.Vertices)
                        {
                            if (vertex.Neighbours.Count == 0)
                            {
                                WriteLine(vertex.ToString(), streamWriter);
                            }
                        }
                        WriteLine("\nTerminating algorithm, since no perfect matching could be found...", streamWriter);
                        break;
                    }
                    // Make some adjustments to avoid boring matchings
                    foreach (var edge in optimalMatching.Edges)
                    {
                        graph.RemoveEdge(edge);
                    }
                    foreach (var vertex in graph.Vertices)
                    {
                        var matchedVersionOfVertex = optimalMatching.Vertices.First(v => v.Index == vertex.Index);
                        var currentMatchedProject = GetProjectFromVertexName(matchedVersionOfVertex.Neighbours[0].Name);
                        RemoveProjectEdgesFromVertex(graph, vertex, currentMatchedProject);
                    }
                }
                Console.WriteLine("-----");
                Console.WriteLine("Done!");
                WriteLine("-----", streamWriter);
                WriteLine("Here are the dates each person gets to go on:", streamWriter);


                // Print the dates for each person, for reference
                foreach (var person in vertexNames)
                {
                    WriteLine(person.ToUpper(), streamWriter);
                    var listOfDates = new List<string>();
                    foreach (var matching in listOfMatchings)
                    {
                        var personVertex = matching.Vertices.First(v => v.Name == person);

                        if (personVertex.Neighbours.Count == 0)
                        {
                            listOfDates.Add("[Unmatched (No project)]");
                        }
                        // Sanity check: Only matched with one person per round!
                        else if (personVertex.Neighbours.Count > 1)
                        {
                            throw new Exception("Somehow, someone got matched with more than one person at once!");
                        }
                        else
                        {
                            listOfDates.Add(personVertex.Neighbours[0].Name);
                        }
                    }

                    foreach (var date in listOfDates)
                    {
                        WriteLine(date, streamWriter);
                    }

                    // Sanity check: no repeat projects!
                    listOfDates.Sort();
                    for (int i = 0; i < listOfDates.Count - 1; i++)
                    {
                        if (GetProjectFromVertexName(listOfDates[i]) == GetProjectFromVertexName(listOfDates[i + 1]))
                        {
                            throw new Exception("Someone got matched with the same project twice!");
                        }
                    }
                    WriteLine(string.Empty,streamWriter);

                }
                Console.WriteLine($"A text file has been placed in {Directory.GetCurrentDirectory()}.");
            }
        }
    }
}