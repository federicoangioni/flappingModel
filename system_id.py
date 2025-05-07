import scipy as sp

mat = sp.io.loadmat("dataset_revision.mat", squeeze_me=True, struct_as_record=False)


def load_data(nexp=None, nrun=None):
    """
    nexp: list
    """

    # Select the experiments to extract
    data = tuple(mat[f"experiment{experiment}"] for experiment in nexp)

    return data


def fit_forces(data):
    pass


print((load_data([98])))
