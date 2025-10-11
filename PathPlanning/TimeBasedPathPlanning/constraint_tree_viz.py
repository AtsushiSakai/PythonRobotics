import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import networkx as nx
from typing import Optional, Dict, Any
from dataclasses import dataclass
from PathPlanning.TimeBasedPathPlanning.ConstraintTree import AgentId, AppliedConstraint, ConstraintTree, ConstraintTreeNode, ForkingConstraint

def visualize_cbs_tree(
    expanded_nodes: Dict[int, ConstraintTreeNode],
    nodes_to_expand: list[ConstraintTreeNode],
    initial_size: int = 15
) -> None:
    """
    Visualize the CBS constraint tree with interactive nodes.
    Click a node to print its details to console.
    """
    
    # Build networkx graph
    G = nx.DiGraph()
    
    # Add all nodes with metadata
    node_colors = []
    node_sizes = []
    node_labels = {}
    
    for idx, node in expanded_nodes.items():
        G.add_node(idx)
        node_labels[idx] = f"<b>Node {idx}</b><br>Cost: {node.cost}<br>Parent: {node.parent_idx}<br>Constraint:<br>{node.constraint}"
        node_colors.append("lightblue")
        node_sizes.append(initial_size)
        
        # Add edge from parent only if parent exists in expanded_nodes
        if node.parent_idx is not None and node.parent_idx in expanded_nodes:
            G.add_edge(node.parent_idx, idx)
            print(f"adding edge btwn {node.parent_idx} and {idx}")

    # Add unexpanded nodes
    # unexpanded_node_map = {}
    # for node in nodes_to_expand:
    #     idx = id(node)  # Use object id for heap nodes
    #     if idx not in G.nodes():
    #         G.add_node(idx)
    #         # node_labels[idx] = f"Node {idx}\n(cost: {node.cost})"
    #         node_labels[idx] = f"<b>Node {idx}</b><br>Cost: {node.cost}<br>Constraint:<br>{node.constraint}"
    #         node_colors.append("lightyellow")
    #         node_sizes.append(initial_size)
    #         unexpanded_node_map[idx] = node
            
    #         if node.parent_idx is not None and node.parent_idx >= 0:
    #             G.add_edge(node.parent_idx, idx)

    # Debug: print all edges in the graph
    print(f"\nAll edges in graph: {list(G.edges())}")
    print(f"All nodes in graph: {list(G.nodes())}")
    
    
    # Use hierarchical layout with fixed horizontal spacing
    # Handle disconnected components
    pos = {}
    x_offset = 0
    for component in nx.weakly_connected_components(G):
        subgraph = G.subgraph(component)
        root = next(iter(subgraph.nodes()))
        component_pos = _hierarchy_pos(subgraph, root=root, vert_gap=0.3, horiz_gap=3.0)
        
        # Offset each component horizontally to avoid overlap
        for node, (x, y) in component_pos.items():
            pos[node] = (x + x_offset, y)
        
        # Increment x_offset for next component
        if len(component_pos) > 0:
            x_offset += max(x for x, y in component_pos.values()) + 3

    print(f"Positions: {pos}")
    
    # Extract edge coordinates with hover text
    edge_x = []
    edge_y = []
    edge_text = []
    for edge in G.edges():
        print(f"Drawing edge: {edge}")
        if edge[0] in pos and edge[1] in pos:
            x0, y0 = pos[edge[0]]
            x1, y1 = pos[edge[1]]
            print(f"  From ({x0}, {y0}) to ({x1}, {y1})")
            edge_x.extend([x0, x1, None])
            edge_y.extend([y0, y1, None])
            edge_text.extend([f"Node {edge[0]} → Node {edge[1]}", f"Node {edge[0]} → Node {edge[1]}", None])
        else:
            print(f"  Edge position not found")
            edge_x.extend([1, 1, None])
            edge_y.extend([5, 5, None])
            edge_text.extend([f"Node {edge[0]} → Node {edge[1]}", f"Node {edge[0]} → Node {edge[1]}", None])

    # Extract node coordinates
    node_x = []
    node_y = []
    node_list = list(G.nodes())
    for node in node_list:
        x, y = 1, 1
        if node in pos:
            x, y = pos[node]
        else:
            print(f"WARNING: Node {node} not in positions!")
        node_x.append(x)
        node_y.append(y)
    
    print(f"Node coordinates: {list(zip(node_list, node_x, node_y))}")
    
    # Create figure
    fig = go.Figure()
    
    # Add edges (visible lines)
    fig.add_trace(go.Scatter(
        x=edge_x, y=edge_y,
        mode='lines',
        line=dict(width=2, color='#888'),
        hoverinfo='none',
        showlegend=False
    ))
    
    # Add invisible thick edges for hover detection
    fig.add_trace(go.Scatter(
        x=edge_x, y=edge_y,
        mode='lines',
        line=dict(width=20, color='rgba(0,0,0,0)'),
        text=edge_text,
        hoverinfo='text',
        showlegend=False
    ))

    # Add nodes
    fig.add_trace(go.Scatter(
        x=node_x, y=node_y,
        mode='markers',
        marker=dict(
            size=node_sizes,
            color=node_colors,
            line=dict(width=2, color='darkblue')
        ),
        text=[node_labels.get(node, f"Node {node}") for node in node_list],
        hoverinfo='text',
        showlegend=False,
        customdata=node_list
    ))
    
    fig.update_layout(
        title="CBS Constraint Tree",
        showlegend=False,
        hovermode='closest',
        margin=dict(b=20, l=5, r=5, t=40),
        xaxis=dict(
            showgrid=False, 
            zeroline=False, 
            showticklabels=False,
            scaleanchor="y",
            scaleratio=1
        ),
        yaxis=dict(
            showgrid=False, 
            zeroline=False, 
            showticklabels=False,
            scaleanchor="x",
            scaleratio=1
        ),
        plot_bgcolor='white',
        autosize=True,
    )
    
    # Add click event
    fig.update_traces(
        selector=dict(mode='markers'),
        customdata=list(G.nodes()),
        hovertemplate='%{text}<extra></extra>'
    )

    fig.update_xaxes(fixedrange=False)
    fig.update_yaxes(fixedrange=False)
    
    # Show and set up click handler
    fig.show()
    
    # Print handler instructions
    print("\nCBS Tree Visualization")
    print("=" * 50)
    print("Hover over nodes to see cost")
    print("Right-click → 'Inspect' → Open browser console")
    print("Then paste this to get node info:\n")
    print("for (let node of document.querySelectorAll('circle')) {")
    print("  node.onclick = (e) => {")
    print("    console.log('Clicked node:', e.target);")
    print("  }")
    print("}\n")
    print("Or use the alternative: Print all nodes programmatically:\n")

def _hierarchy_pos(G, root=None, vert_gap=0.2, horiz_gap=1.0, xcenter=0.5):
    """
    Create hierarchical layout for tree visualization with fixed horizontal spacing.
    Spreads nodes wide at each level to avoid overlaps.
    """
    if not nx.is_tree(G):
        G = nx.DiGraph(G)
    
    def _hierarchy_pos_recursive(G, root, vert_gap=0.2, horiz_gap=1.0, xcenter=0.5, pos=None, parent=None, child_index=0, level_nodes=None):
        if pos is None:
            pos = {root: (xcenter, 0)}
            level_nodes = {0: [root]}
        else:
            level = pos[parent][1] / (-vert_gap)
            pos[root] = (xcenter, pos[parent][1] - vert_gap)
            if int(level) + 1 not in level_nodes:
                level_nodes[int(level) + 1] = []
            level_nodes[int(level) + 1].append(root)
        
        neighbors = list(G.neighbors(root))
        
        if len(neighbors) != 0:
            num_children = len(neighbors)
            # Spread children very wide to avoid any overlap
            spread = num_children * horiz_gap * 2
            start_x = xcenter - spread / 2
            for i, neighbor in enumerate(neighbors):
                nextx = start_x + i * (spread / max(num_children - 1, 1))
                _hierarchy_pos_recursive(G, neighbor, vert_gap=vert_gap, horiz_gap=horiz_gap,
                                        xcenter=nextx, pos=pos, parent=root, child_index=i, level_nodes=level_nodes)
        
        return pos
    
    return _hierarchy_pos_recursive(G, root, vert_gap, horiz_gap, xcenter)


# Example usage:
if __name__ == "__main__":
    from dataclasses import dataclass
    from typing import Optional
    
    @dataclass
    class MockConstraint:
        agent: int
        time: int
        location: tuple
        
        def __repr__(self):
            return f"Constraint(agent={self.agent}, t={self.time}, loc={self.location})"
    
    @dataclass
    class MockNode:
        parent_idx: Optional[int]
        constraint: Optional[MockConstraint]
        paths: dict
        cost: int
    
    # Create mock tree
    nodes = {
        0: MockNode(None, None, {"a": [], "b": []}, 10),
        1: MockNode(0, MockConstraint(0, 2, (0, 0)), {"a": [(0,0), (1,0)], "b": [(0,1), (0,2)]}, 12),
        2: MockNode(0, MockConstraint(1, 1, (0,1)), {"a": [(0,0), (1,0)], "b": [(0,1), (0,2)]}, 11),
        3: MockNode(1, MockConstraint(0, 3, (1,0)), {"a": [(0,0), (1,0), (1,1)], "b": [(0,1), (0,2)]}, 14),
        4: MockNode(2, None, {"a": [(0,0), (1,0)], "b": [(0,1), (1,1)]}, 12),
    }
    
    visualize_cbs_tree(nodes, [])