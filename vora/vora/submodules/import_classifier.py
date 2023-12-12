import pickle

def load_classifier():
    model_name = "new3s.p"
    model_dict = pickle.load(open(model_name, "rb"))
    model = model_dict["model"]
    return model