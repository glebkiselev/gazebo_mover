from planner.util import *
import theano
import theano.tensor as T
from lasagne.layers import *
from lasagne.nonlinearities import linear
from lasagne.init import GlorotNormal
import math


class Network:
    def __init__(self, n_actions, observation_shape):
        observations = T.matrix(name="obs")
        actions = T.matrix(name="action")
        cummulative_returns = T.vector(name="G = r + gamma*r' + gamma^2*r'' + ...")
        old_m = T.matrix(name="action probabilities from previous iteration m")
        old_logstd = T.matrix(name="action probabilities from previous iteration sigma")
        all_inputs = [observations, actions, cummulative_returns, old_m, old_logstd]
        nn = InputLayer((None,) + observation_shape, input_var=observations)
        nn1 = DenseLayer(nn, 128, W=GlorotNormal())
        nn2 = DenseLayer(nn1, 64, W=GlorotNormal())
        m = DenseLayer(nn2, n_actions, nonlinearity=linear)
        logsigma = DenseLayer(nn2, n_actions, nonlinearity=linear)

        #-----------------------------------------------
        mean = get_output([m])[0]
        logstd = get_output([logsigma])[0]
        self.weights = get_all_params([m, logsigma], trainable=True)
        self.get_mean = theano.function([observations], mean, allow_input_downcast=True)
        self.get_logstd = theano.function([observations], logstd, allow_input_downcast=True)

        #LOSS
        log_p = log_probability(logstd, mean, actions)
        oldlog_p = log_probability(old_logstd, old_m, actions)
        L_surr = -T.sum(T.exp(log_p - oldlog_p) * cummulative_returns)
        kl = kl_sym(old_m, mean, old_logstd, logstd)
        entropy = -T.sum(logstd + 0.5 * np.log(2 * math.pi * math.e))
        self.compute_losses = theano.function(all_inputs, [L_surr, kl, entropy], allow_input_downcast=True)
        self.get_surrogate_gradients = theano.function(all_inputs, get_flat_gradient(L_surr, self.weights),
                                                       allow_input_downcast=True)

        #FISHER VECTOR PRODUCT
        conjugate_grad_intermediate_vector = T.vector("intermediate grad in conjugate_gradient")
        weight_shapes = [var.get_value().shape for var in self.weights]
        tangents = slice_vector(conjugate_grad_intermediate_vector, weight_shapes)
        kl_firstfixed = kl_sym_firstfixed(mean, logstd)
        gradients = T.grad(kl_firstfixed, self.weights)
        gradient_vector_product = [T.sum(g * t) for (g, t) in zip(gradients, tangents)]
        fisher_vector_product = get_flat_gradient(sum(gradient_vector_product), self.weights)
        self.compute_fisher_vector_product = theano.function([observations, conjugate_grad_intermediate_vector],
                                                             fisher_vector_product, allow_input_downcast=True)

        #Function that exports network weights as a vector
        flat_weights = T.concatenate([var.ravel() for var in self.weights])
        self.get_flat_weights = theano.function([], flat_weights)

        #Function that imports vector back into network weights
        flat_weights_placeholder = T.vector("flattened weights")
        assigns = slice_vector(flat_weights_placeholder, weight_shapes)
        self.load_flat_weights = theano.function([flat_weights_placeholder], updates=dict(zip(self.weights, assigns)))