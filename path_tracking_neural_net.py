import numpy as np
import csv


def nonlin(x, deriv=False):
    # sigmoid activation function:
    if deriv is True:
        return x * (1 - x)
    return 1 / (1 + np.exp(- x))


def one_layer(X, y):
    # initialize weights randomly with mean 0
    syn0 = 2 * np.random.random((2, len(X))) - 1
    syn1 = 2 * np.random.random((len(y), 2)) - 1
    l1 = []
    l2 = []

    for i in range(1000):

        print("Iteration: ", i)

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

    return syn0, syn1


def zero_layers(X, y):
    np.random.seed(1)

    syn0 = 2 * np.random.random((4, 2)) - 1

    for iter in range(60000):
        # forward propagation
        l0 = X
        l1 = nonlin(np.dot(l0, syn0))

        l1_error = y - l1

        l1_delta = l1_error * nonlin(l1, True)

        syn0 += np.dot(l0.T, l1_delta)
        print("Output after training: ", l1)
        print("weights: ", syn0)
        return syn0


def read_csv_files(start, end):
    # input and output data sets:
    X = []
    y = []
    for k in range(start, end):
        f = 'data_set' + str(k + 1) + '.csv'
        with open(f, 'r') as csvfile:
            rdr = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in rdr:
                for i in range(len(row)):
                    row[i] = float(row[i])
                X.append(row[2:4])
                y.append(row[4:6])
    X = np.array(X)
    y = np.array(y)
    return X, y


X, y = read_csv_files(0, 7)
print("X: ", X)
print("y: ", y)
syn0, syn1 = one_layer(X, y)
print("syn0: ", syn0)
print("syn1: ", syn1)

x_test, y_test = read_csv_files(6, 7)
print("x_test: ", x_test)

l0 = x_test
l1 = nonlin(np.dot(l0, syn0))
l2 = nonlin(np.dot(l1, syn1))

print("l1: ", l1)
print("prediction: ", l2)
print("y_test: ", y_test)

# find error:
error = []
for i in range(len(l2)):
    error.append(abs(l2[i][0] - y_test[i][0]))
print("error: ", error)