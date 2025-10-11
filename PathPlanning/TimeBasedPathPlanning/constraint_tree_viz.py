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
        
        # Add edge from parent
        # if node.parent_idx is not None:
        if node.parent_idx is not None and node.parent_idx in expanded_nodes:
            G.add_edge(node.parent_idx, idx)
            # G.add_edge(0, 5)
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

    # Use hierarchical layout with fixed horizontal spacing
    pos = _hierarchy_pos(G, root=next(iter(G.nodes()), None), vert_gap=0.3, horiz_gap=1.5)

    # Extract edge coordinates
    edge_x = []
    edge_y = []
    for edge in G.edges():
        print(f"Drawing edge: {edge}")
        if edge[0] in pos and edge[1] in pos:
            x0, y0 = pos[edge[0]]
            x1, y1 = pos[edge[1]]
            edge_x.extend([x0, x1, None])
            edge_y.extend([y0, y1, None])
        else:
            edge_x.extend([1, 1, None])
            edge_y.extend([5, 5, None])
    
    # Extract node coordinates
    node_x = []
    node_y = []
    for node in G.nodes():
        x, y = 1, 1
        if node in pos:
            x, y = pos[node]
        node_x.append(x)
        node_y.append(y)
    
    # Create figure
    fig = go.Figure()
    
    # Add edges
    fig.add_trace(go.Scatter(
        x=edge_x, y=edge_y,
        mode='lines',
        line=dict(width=2, color='#888'),
        hoverinfo='none',
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
        text=[node_labels[node] for node in G.nodes() if node in node_labels],
        hoverinfo='text',
        showlegend=False,
        customdata=list(G.nodes())
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
    """
    if not nx.is_tree(G):
        G = nx.DiGraph(G)
    
    def _hierarchy_pos_recursive(G, root, vert_gap=0.2, horiz_gap=1.0, xcenter=0.5, pos=None, parent=None, child_index=0):
        if pos is None:
            pos = {root: (xcenter, 0)}
        else:
            pos[root] = (xcenter, pos[parent][1] - vert_gap)
        
        neighbors = list(G.neighbors(root))
        
        if len(neighbors) != 0:
            num_children = len(neighbors)
            # Spread children horizontally with fixed gap
            start_x = xcenter - (num_children - 1) * horiz_gap / 2
            for i, neighbor in enumerate(neighbors):
                nextx = start_x + i * horiz_gap
                _hierarchy_pos_recursive(G, neighbor, vert_gap=vert_gap, horiz_gap=horiz_gap,
                                        xcenter=nextx, pos=pos, parent=root, child_index=i)
        
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