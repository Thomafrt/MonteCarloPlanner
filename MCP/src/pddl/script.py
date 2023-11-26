import pandas as pd
import matplotlib.pyplot as plt


def process_domain(data, domain_data, domain_name):
    for i in data.index:
        print(f"Processing row {i} with domain: {data.domain[i]}")
        if data.domain[i] == domain_name:
            domain_data["times_MCP"].append(data.MCP_time[i])
            domain_data["steps_MCP"].append(data.MCP_length[i])
            domain_data["times_HSP"].append(data.HSP_time[i])
            domain_data["steps_HSP"].append(data.HSP_length[i])
            domain_data["problems"].append(str(data.problem_number[i]))
            print(f"Data appended for {domain_name}.")


def plot_graph(title, x_label, y_label, problems, times_MCP, times_HSP, filename):
    sorted_times = sorted(zip(times_HSP, times_MCP, problems))
    times_HSP, times_MCP, problems = zip(*sorted_times)

    plt.figure()
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title)
    plt.plot(problems, times_MCP, label="MCP")
    plt.plot(problems, times_HSP, label="HSP")
    plt.legend()
    plt.savefig(filename)


print("Data read from CSV:")
data = pd.read_csv("data.csv", on_bad_lines="skip")
print(data.head())

# Initialize domain data
blocks_data = {
    "times_MCP": [],
    "steps_MCP": [],
    "times_HSP": [],
    "steps_HSP": [],
    "problems": [],
}
logistics_data = {
    "times_MCP": [],
    "steps_MCP": [],
    "times_HSP": [],
    "steps_HSP": [],
    "problems": [],
}
gripper_data = {
    "times_MCP": [],
    "steps_MCP": [],
    "times_HSP": [],
    "steps_HSP": [],
    "problems": [],
}

# Process domains
process_domain(data, blocks_data, "domain")
process_domain(data, gripper_data, "_domain")
process_domain(data, logistics_data, "cs_domain")

# Plot graphs
plot_graph(
    "Resolution time of MCP and HSP planners in blocksworld",
    "Problem number",
    "Resolution time",
    blocks_data["problems"],
    blocks_data["times_MCP"],
    blocks_data["times_HSP"],
    "graphRuntimeBlocks.png",
)

plot_graph(
    "Number of steps needed by MCP and HSP planners to resolve problem in blocksworld",
    "Problem number",
    "Number of steps to resolve",
    blocks_data["problems"],
    blocks_data["steps_MCP"],
    blocks_data["steps_HSP"],
    "graphNumberOfStepsBlocks.png",
)

plot_graph(
    "Resolution time of MCP and HSP planners in logistics",
    "Problem number",
    "Resolution time",
    logistics_data["problems"],
    logistics_data["times_MCP"],
    logistics_data["times_HSP"],
    "graphRuntimeLogistics.png",
)

plot_graph(
    "Number of steps needed by MCP and HSP planners to resolve problem with logistics",
    "Problem number",
    "Number of steps to resolve",
    logistics_data["problems"],
    logistics_data["steps_MCP"],
    logistics_data["steps_HSP"],
    "graphNumberOfStepsLogistics.png",
)

plot_graph(
    "Resolution time of MCP and HSP planners in gripper",
    "Problem number",
    "Resolution time",
    gripper_data["problems"],
    gripper_data["times_MCP"],
    gripper_data["times_HSP"],
    "graphRuntimeGripper.png",
)

plot_graph(
    "Number of steps needed by MCP and HSP planners to resolve problem with gripper",
    "Problem number",
    "Number of steps to resolve",
    gripper_data["problems"],
    gripper_data["steps_MCP"],
    gripper_data["steps_HSP"],
    "graphNumberOfStepsGripper.png",
)
