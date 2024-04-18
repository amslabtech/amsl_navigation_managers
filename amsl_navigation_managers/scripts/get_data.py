import yaml


def from_yaml(filename):
    with open(filename, "r") as f:
        data = yaml.safe_load(f)

    labels = []
    ids = []

    for node in data["NODE"]:
        if node["type"] == "landmark":
            ids.append(node["id"])
            labels.append(node["label"])

    return ids, labels


if __name__ == "__main__":
    filename = "/home/user/ws/src/advanced_navigation_launcher/map/graph/ikuta_graph.yaml"
    ids = []
    labels = []
    choices = ""

    ids, labels = from_yaml(filename)

    for i in range(len(ids)):
        choices += str(ids[i]) + ". " + labels[i] + ", "

    # print(type(ids[0]), ids[0])
    # print(type(labels[0]))

    print(choices)
