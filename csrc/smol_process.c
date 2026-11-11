#include "smol_model.h"
// #include "stdio.h"

#ifdef FIPPLA
#define WH_KERNEL(kernel_size,c, o,k,m) wh_kernel[c*(c*o+k)+m]
#else
#define WH_KERNEL(kernel_size,c, o,k,m) wh_kernel[kernel_size*(k*c+m)+o]
#endif

bool process_model(WaveNetModel *la, float *input, float *output, size_t n_samples, float *history, size_t *history_count, float *restrict scratch, float in_gain, float out_gain) {
  float *wh = la->weights;

  uint32_t history_off = (uint32_t)*history_count; // truncated to 32 bit

  // first layer: input is from upscale
  float *layer_in = history + la->history_offset[0];
  uint32_t in_mask = la->time_mask[0];

  float *headsum = history + la->history_offset[la->n_layers];
  uint32_t headsum_mask = la->time_mask[la->n_layers];

  int c = la->n_chan;

  {
    float *wh_upscale = wh+la->whpos_upscale;
    for (size_t n = 0; n < n_samples; n++) {
      size_t t = (n+history_off) & in_mask;
      for (size_t k = 0; k < c; k++) {
        layer_in[t*c+k] = input[n]*wh_upscale[k]*in_gain;
      }
    }
  }

  {
    for (size_t n = 0; n < n_samples; n++) {
      size_t t = (n+history_off) & headsum_mask;
      for (size_t k = 0; k < c; k++) {
        headsum[t*c+k] = 0;
      }
    }
  }


  for (size_t i = 0; i < la->n_layers; i++) {
    float *layer_out = history + la->history_offset[i+1];
    uint32_t out_mask = la->time_mask[i+1];
    uint32_t kernel_size = la->kernel_sizes[i];
    uint32_t dilation = la->dilations[i];

    float *wh_kernel = wh+la->whpos_layer[i].conv_kernel;
    float *wh_bias = wh+la->whpos_layer[i].conv_bias;

    for (size_t n = 0; n < n_samples; n++) {
      for (size_t k = 0; k < c; k++) {
        scratch[n*c+k] = wh_bias[k];
      }
    }

    if (c == 3) {
#define C 3
      for (size_t o = 0; o < kernel_size; o++) {
        size_t offset = (kernel_size-1-o)*dilation;
        for (size_t n = 0; n < n_samples; n++) {
          size_t t_in = (n+history_off-offset) & in_mask;
          for (size_t k = 0; k < C; k++) {
            for (size_t m = 0; m < C; m++) {
              // obviously transpose wh_kernel:p
              scratch[n*c+k] += layer_in[t_in*c+m]*WH_KERNEL(kernel_size,C, o,k,m);
            }
          }
        }
      }
#undef C
    } else {
      for (size_t o = 0; o < kernel_size; o++) {
        size_t offset = (kernel_size-1-o)*dilation;
        for (size_t n = 0; n < n_samples; n++) {
          size_t t_in = (n+history_off-offset) & in_mask;
          for (size_t k = 0; k < c; k++) {
            for (size_t m = 0; m < c; m++) {
              // obviously transpose wh_kernel:p
              scratch[n*c+k] += layer_in[t_in*c+m]*WH_KERNEL(kernel_size,c, o,k,m);
            }
          }
        }
      }
    }

    float *wh_mixin = wh+la->whpos_layer[i].mixin;
    for (size_t n = 0; n < n_samples; n++) {
      size_t t_head = (n+history_off) & headsum_mask;
      for (size_t k = 0; k < c; k++) {
        float mixed = scratch[n*c+k]+wh_mixin[k]*in_gain*input[n];
#ifdef ACTIVATION_HISTORY
        la->activation_history[i][k] += (mixed > 0);
#endif
        // TODO: check config for 0.01 ..
        scratch[n*c+k] = mixed > 0 ? mixed : 0.01 * mixed;
        headsum[t_head*c+k] += scratch[n*c+k];
      }
    }

    // for last layer, layer_out is not used and aliases headsum, do not write!
    if (i < la->n_layers-1) {
      float *wh_layer1x1 = wh+la->whpos_layer[i].layer1x1_kernel;
      float *wh_layer1x1_bias = wh+la->whpos_layer[i].layer1x1_bias;
      for (size_t n = 0; n < n_samples; n++) {
        size_t t_in = (n+history_off) & in_mask;
        size_t t_out = (n+history_off) & out_mask;
        for (size_t k = 0; k < c; k++) {
          layer_out[t_out*c+k] = wh_layer1x1_bias[k];
          for (size_t m = 0; m < c; m++) {
            layer_out[t_out*c+k] += scratch[n*c+m]*wh_layer1x1[k*c+m];
          }
          layer_out[t_out*c+k] += layer_in[t_in*c+k];
        }
      }
    }

    // begehen
    layer_in = layer_out;
    in_mask = out_mask;
  }

  {
    float head_bias = wh[la->whpos_headsum_bias];
    for (size_t n = 0; n < n_samples; n++) {
      output[n] = head_bias;
    }
  }

  {
    uint32_t kernel_size = la->head_kernel_size;
    float *wh_headsum = wh+la->whpos_headsum_kernel;
    for (size_t o = 0; o < kernel_size; o++) {
      for (size_t n = 0; n < n_samples; n++) {
          size_t t_head = (n+history_off-o) & headsum_mask;
          for (size_t k = 0; k < c; k++) {
            output[n] += headsum[t_head*c+k]*wh_headsum[kernel_size*k+o];
          }
      }
    }
  }

  {
    float real_gain = la->head_scale*out_gain;
    for (size_t n = 0; n < n_samples; n++) {
      output[n] *= real_gain;
    }
  }

  *history_count += n_samples;

  return true;
}

