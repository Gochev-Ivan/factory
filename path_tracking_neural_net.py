import numpy as np
import csv


def nonlin(x, deriv=False):
    # sigmoid activation function:
    if deriv is True:
        return x * (1 - x)
    return 1 / (1 + np.exp(- x))


def two_layers(X, y):
    # initialize weights randomly with mean 0
    syn0 = 2 * np.random.random((4, len(X))) - 1
    syn1 = 2 * np.random.random((len(y), 2)) - 1
    l2 = []

    for i in range(60000):
        # forward propagation:
        l0 = X
        l1 = nonlin(np.dot(l0, syn0))
        l2 = nonlin(np.dot(l1, syn1))

        l2_error = y - l2

        if (i % 10000) == 0:
            print("Error:" + str(np.mean(np.abs(l2_error))))

        l2_delta = l2_error * nonlin(l2, deriv=True)

        l1_error = l2_delta.dot(syn1.T)

        l1_delta = l1_error * nonlin(l1, deriv=True)

        # update weights:
        syn1 += l1.T.dot(l2_delta)
        syn0 += l0.T.dot(l1_delta)

    print("Output after training: ", l2)
    return l2


def one_layer():
    # ==========
    X = []
    y = []
    for k in range(1, 4):
        f = 'data_set' + str(k) + '.csv'
        with open(f, 'r') as csvfile:
            rdr = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in rdr:
                for i in range(len(row)):
                    row[i] = float(row[i])
                X.append(row[:4])
                y.append(row[4:6])
    X = np.array(X)
    y = np.array(y)

    np.random.seed(1)
    # ==========

    syn0 = 2 * np.random.random((4, 2)) - 1

    for iter in range(60000):
        # forward propagation
        l0 = X
        l1 = nonlin(np.dot(l0, syn0))

        l1_error = y - l1

        l1_delta = l1_error * nonlin(l1, True)

        syn0 += np.dot(l0.T, l1_delta)
        print("Output after training: ", l1)
        return l1


# input and output data sets:
X = []
y = []
for k in range(1, 4):
    f = 'data_set' + str(k) + '.csv'
    with open(f, 'r') as csvfile:
        rdr = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in rdr:
            for i in range(len(row)):
                row[i] = float(row[i])
            X.append(row[:4])
            y.append(row[4:6])
X = np.array(X)
y = np.array(y)

np.random.seed(1)

# one_layer(X, y)
# two_layers(X, y)