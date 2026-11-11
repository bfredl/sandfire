#include <stdint.h>
// #include <string.h>
#include <stdbool.h>
#include <stddef.h>

static inline void* memcpy(void* restrict dest, const void* restrict src, size_t n) {
    unsigned char* d = (unsigned char*)dest;
    const unsigned char* s = (const unsigned char*)src;
    while (n--) {
        *d++ = *s++;
    }
    return dest;
}

typedef struct {
  uint8_t version_major;
  uint8_t version_minor;
  uint8_t version_patch;
  uint8_t meta_flags;
  double sample_rate;
  double loudness;
  double input_level;
  double output_level;
} MetadataBlock;

static const uint8_t GATING_NONE = 0;
static const uint8_t GATING_GATED = 1;
static const uint8_t GATING_BLENDED = 2;

// #define ACTIVATION_HISTORY
#define FIPPLA

#define MAX_LAYERS 23
#define MAX_CHANNELS 8
#define MAX_WEIGHT_COUNT 1871

typedef struct {
  MetadataBlock metadata;

  int head_kernel_size; ///< Kernel size of head rechannel convolution (>= 1)
  int n_chan; ///< Number of channels in each layer (== bottleneck)
  int n_layers;
  uint32_t dilations[MAX_LAYERS];
  uint16_t kernel_sizes[MAX_LAYERS]; ///< Per-layer kernel sizes, one per layer
  bool head_bias; ///< Whether to use bias in head rechannel

  // indicies into a weight array
  uint32_t whpos_upscale; // [input_size, n_chan]
  struct {
    // TODO: check ordning in actual use
    uint32_t conv_kernel; // [kernel_sizes, n_chan, n_chan]
    uint32_t conv_bias; // [n_chan]

    uint32_t mixin;// [condition_size, n_chan]

    uint32_t layer1x1_kernel; // [n_chan, n_chan]
    uint32_t layer1x1_bias; // [n_chan]

  } whpos_layer[MAX_LAYERS];
  uint32_t whpos_headsum_kernel;
  uint32_t whpos_headsum_bias;

  uint32_t history_offset[MAX_LAYERS+1];
  uint32_t time_mask[MAX_LAYERS+1];

#ifdef ACTIVATION_HISTORY
  uint32_t activation_history[MAX_LAYERS][MAX_CHANNELS];
#endif

  uint32_t history_size;
  float head_scale;
  float weights[MAX_WEIGHT_COUNT];
  uint32_t prewarm_len;
} WaveNetModel;

bool read_model(WaveNetModel *res, const uint8_t* data, size_t size, size_t max_samples);
bool process_model(WaveNetModel *mod, float *input, float *output, size_t n_samples, float *history, size_t *history_count, float *scratch, float in_gain, float out_gain);
