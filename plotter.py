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
        # for i in range(var_num):
        #    data[i].append(float(split_l[1 + 2*i]))
        data[0].append(float(split_l[1 + 2*0]))
        data[1].append(float(split_l[1 + 2*1]))
        data[2].append(float(split_l[1 + 2*2]))
        data[3].append(float(split_l[1 + 2*3]))
        data[4].append(float(split_l[1 + 2*4]))
        data[5].append(float(split_l[1 + 2*5]))
        data[6].append(float(split_l[1 + 2*6]))
        data[7].append(float(split_l[1 + 2*7]))
        data[8].append(float(split_l[1 + 2*8]))
        data[9].append(float(split_l[1 + 2*9]))

    plt.figure(figsize=(10,10))
    plt.subplot(211)
    for i in [3, 4]:
        plt.plot(range(len(content)), data[i], label=labels[i])
    plt.title(title)
    plt.xlim([0, len(content)])
    plt.ylim([-12, 12])
    plt.grid()
    plt.legend()
    
    plt.subplot(212)
    for i in [8, 9]:
        plt.plot(range(len(content)), data[i], label=labels[i])
    plt.grid()
    plt.xlim([0, len(content)])
    plt.ylim([-12, 12])
    plt.legend()

    # plt.subplot(313)
    # for i in [0, 5]:
    #     plt.plot(range(len(content)), data[i], label=labels[i])
    # plt.grid()
    # plt.legend()

    plt.xlabel(x)
    plt.ylabel(y)
    
    plt.savefig(save_file)

plot_data(
    load_file='week9_data/tuning.txt',
    save_file='week9_data/tuning.png',
    title="Milestone 2",
    x="t",
    y="")