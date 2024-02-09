import matplotlib.pyplot as plt
import networkx as nx

def readGraph(filepath):
    G = nx.Graph()
    with open(filepath, 'r') as file:
        for line in file:
            data = line.split(" ")
            G.add_edge(int(data[0]), int(data[1]))
    return G

def readPosition(filepath):
    pos = {}
    idx = 0
    with open(filepath, 'r') as file:
        for line in file:
            data = line.split(" ")
            pos[idx] = (float(data[0]), float(data[1]))
            idx += 1
    return pos

def main():
    #G = readGraph('/home/michbaum/Projects/optag_EH/data/floorplan/edges.txt')
    G = readGraph('/home/michbaum/Projects/optag_EH/data/floorplan/path.txt')
    pos = readPosition('/home/michbaum/Projects/optag_EH/data/floorplan/nodes.txt')
    nx.draw(G, pos=pos,with_labels=True, font_weight='bold')
    plt.show()

if __name__ == "__main__":
    main()