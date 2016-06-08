#pragma once

//Functions Implemented
extern "C" void *load_network(const char *filename);
extern "C" int   num_inputs(void *network);
extern "C" int   num_outputs(void *network);
extern "C" int   evaluate_network(void *network, float *input, float *output);
extern "C" void  destroy_network(void *network);
