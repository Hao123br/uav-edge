from tensorflow import keras
from glob import glob
import numpy as np


def read_mobility(user_filename):
    return np.genfromtxt(user_filename, delimiter=" ")


def create_dataset(dataset, look_back=10):
    dataX, dataY = [], []
    for i in range(len(dataset)-look_back-1):
        a = dataset[i:(i+look_back)]
        dataX.append(a)
        dataY.append(dataset[i + look_back])
    return np.array(dataX), np.array(dataY)


def main():
    data_dir = "san_francisco/*"
    output_dir = "models/"

    num_users = 20
    num_samples = 1000
    user_filenames = glob(data_dir)[:num_users]

    for userid, u in enumerate(user_filenames):
        data = read_mobility(u)[:num_samples]
        train_x, train_y = create_dataset(data)

        model = create_model()
        model.fit(train_x, train_y, epochs=50)

        stripped_name = output_dir + "".join(u.split("/")[1:])
        model.save(stripped_name)


def make_prediction(userid, time):
    """ the first thing to do is to translate a user id into a filename for the model to execute, 
        in the gen_mobility.py, we should generate a relation between id and filename to be read here.
    """
    pass

def create_model(learning_rates=0.001, n_layers=1, n_lstm_cells=20,
                 n_dense_neurons=100, droput_rate=0.15):
    """ instantiate non-trained model """

    # initializer = keras.initializers.RandomUniform(
    # minval=-0.05, maxval=0.05, seed=1234)
    model = keras.Sequential()
    model.add(keras.layers.LSTM(n_lstm_cells,
            input_shape=(10, 2),
            dropout=droput_rate,
            recurrent_dropout=droput_rate))
    model.add(keras.layers.LeakyReLU())
    for i in range(n_layers):
        model.add(keras.layers.Dense(n_dense_neurons,
                activation="relu"))
        model.add(keras.layers.Dropout(droput_rate))
        model.add(keras.layers.LeakyReLU())

    model.add(keras.layers.Dense(2, activation="relu"))
    model.add(keras.layers.LeakyReLU())

    model.compile(
        loss="mean_squared_logarithmic_error",
        metrics=["mean_squared_logarithmic_error"],
        optimizer=keras.optimizers.Adam(
            learning_rate=learning_rates))

    return model


if __name__ == "__main__":
    main()
