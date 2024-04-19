import matplotlib.pyplot as plt

def plot_data(load_file, save_file, title, x, y):
    file = open(load_file, "r")
    content = file.readlines()
    
    var_num = len(content[0].split(" ")) // 2
    labels = []
    for i in range(var_num):
        labels.append(content[0].split(' ')[2 * i])
    
    data = [[] for _ in range(var_num)]
    data.append([])
    for l in content:
        split_l = l.replace('\n', '').split(' ')
        for i in range(var_num):
           data[i].append(float(split_l[1 + 2*i]))
        data[5].append(0)

    plt.figure(figsize=(10,10))
    plt.subplot(211)
    for i in [1, 0, 2]:
        plt.plot(range(len(content)), data[i], label=labels[i])
    plt.plot(range(len(content)), data[5], label=" Set Point")
    plt.title(title)
    plt.legend()
    
    plt.subplot(212)
    for i in [3, 4]:
        plt.plot(range(len(content)), data[i], label=labels[i])
    plt.legend()

    plt.xlabel(x)
    plt.ylabel(y)
    
    plt.savefig(save_file)

plot_data(
    load_file='week4_data/stretchmilestone.txt',
    save_file='week4_data/stretchmilestone.png',
    title="Milestone 4",
    x="t",
    y="")