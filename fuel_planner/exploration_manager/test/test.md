  // Test graph search
  std::cout << "1" << std::endl;
  GraphSearch<ViewNode> graph_search;

  std::cout << "2" << std::endl;
  // Start node
  ViewNode::Ptr v0(new ViewNode(Vector3d(0, 0, 0), 0));
  // First layer
  ViewNode::Ptr v10(new ViewNode(Vector3d(1, 0, 0), -1));
  ViewNode::Ptr v11(new ViewNode(Vector3d(1, 0, 0), 0));
  ViewNode::Ptr v12(new ViewNode(Vector3d(1, 0, 0), 1));
  std::cout << "3" << std::endl;
  // Second layer
  ViewNode::Ptr v20(new ViewNode(Vector3d(2, 0, 0), -1));
  ViewNode::Ptr v21(new ViewNode(Vector3d(2, 0, 0), 0));
  ViewNode::Ptr v22(new ViewNode(Vector3d(2, 0, 0), 1));
  // End node
  ViewNode::Ptr v3(new ViewNode(Vector3d(3, 0, 0), 0));

  std::cout << "4" << std::endl;
  // Add nodes
  graph_search.addNode(v0);
  graph_search.addNode(v10);
  graph_search.addNode(v11);
  graph_search.addNode(v12);
  graph_search.addNode(v20);
  graph_search.addNode(v21);
  graph_search.addNode(v22);
  graph_search.addNode(v3);

  std::cout << "5" << std::endl;
  // Add edges
  graph_search.addEdge(v0->id_, v10->id_);
  graph_search.addEdge(v0->id_, v11->id_);
  graph_search.addEdge(v0->id_, v12->id_);

  graph_search.addEdge(v10->id_, v20->id_);
  graph_search.addEdge(v10->id_, v21->id_);
  graph_search.addEdge(v10->id_, v22->id_);
  graph_search.addEdge(v11->id_, v20->id_);
  graph_search.addEdge(v11->id_, v21->id_);
  graph_search.addEdge(v11->id_, v22->id_);
  graph_search.addEdge(v12->id_, v20->id_);
  graph_search.addEdge(v12->id_, v21->id_);
  graph_search.addEdge(v12->id_, v22->id_);

  graph_search.addEdge(v20->id_, v3->id_);
  graph_search.addEdge(v21->id_, v3->id_);
  graph_search.addEdge(v22->id_, v3->id_);

  std::cout << "6" << std::endl;
  // Search path
  vector<ViewNode::Ptr> path;
  graph_search.DijkstraSearch(v0->id_, v3->id_, path);
  std::cout << "path size: " << path.size() << std::endl;
  for (int i = 0; i < path.size(); ++i)
  {
    std::cout << "node id: " << path[i]->id_ << std::endl;
  }


  PROBLEM_FILE = /home/boboyu/workspaces/plan_ws/src/fast_planner/utils/lkh_tsp_solver/resource/test.tsp
GAIN23 = NO 
OUTPUT_TOUR_FILE = /home/boboyu/workspaces/plan_ws/src/fast_planner/utils/lkh_tsp_solver/resource/test.txt
RUNS = 1