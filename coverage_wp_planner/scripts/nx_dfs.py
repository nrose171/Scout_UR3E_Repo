import pandas as pd
import numpy as np
import networkx as nx

# Create a 10x10 grid of points
x = np.linspace(0, 9, 10)
y = np.linspace(0, 9, 10)
xx, yy = np.meshgrid(x, y)

# Flatten the arrays to create a list of xy coordinates
xy_coords = np.vstack((xx.ravel(), yy.ravel())).T

# Create a pandas DataFrame from the xy coordinates
df = pd.DataFrame(xy_coords, columns=['x', 'y'])

# Create a graph from the DataFrame
G = nx.Graph()

# Add nodes to the graph
for index, row in df.iterrows():
    G.add_node(index, pos=(row['x'], row['y']))

# # Add edges between adjacent points
# for i in range(len(df)):
#     for j in range(i+1, len(df)):
#         if (abs(df.iloc[i]['x'] - df.iloc[j]['x']) == 1 and df.iloc[i]['y'] == df.iloc[j]['y']) or \
#            (abs(df.iloc[i]['y'] - df.iloc[j]['y']) == 1 and df.iloc[i]['x'] == df.iloc[j]['x']):
#             G.add_edge(i, j)

# Perform DFS on the graph
print("Depth-First Search (DFS) traversal:")
# for node in nx.dfs_preorder_nodes(G):
#     print(node)
print(nx.dfs_preorder_nodes(G))
print(list(nx.dfs_preorder_nodes(G)))

import matplotlib.pyplot as plt

plt.plot(df['x'].values[list(nx.dfs_preorder_nodes(G))], df['y'].values[list(nx.dfs_preorder_nodes(G))], "x-")
plt.show()