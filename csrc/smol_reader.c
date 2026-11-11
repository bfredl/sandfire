#include "smol_model.h"

// Magic number: "NAMB" as little-endian uint32
static const uint32_t NAMB_MAGIC = 0x4E414D42;
static const uint16_t NAMB_FORMAT_VERSION = 1;

// static const uint8_t ARCH_LINEAR = 0;
// static const uint8_t ARCH_CONVNET = 1;
// static const uint8_t ARCH_LSTM = 2;
static const uint8_t ARCH_WAVENET = 3;

// File offsets
static const size_t FILE_HEADER_SIZE = 32;
#define METADATA_BLOCK_SIZE 48

#define HUNK(name, type) \
static type name(const unsigned char *data, size_t pos) { \
  type res; \
  memcpy(&res, data+pos, sizeof(type)); \
  return res; \
} \

HUNK(boke, uint32_t)
HUNK(pull, uint16_t)
HUNK(take, uint8_t)
HUNK(slurp, double)
HUNK(collect, float)


void read_metadata(MetadataBlock *meta, const unsigned char data[METADATA_BLOCK_SIZE]) {
  meta->version_major = take(data, 0);
  meta->version_minor = take(data, 1);
  meta->version_patch = take(data, 2);
  meta->meta_flags = take(data, 3);
  meta->sample_rate = slurp(data, 4);
  meta->loudness = slurp(data, 12);
  meta->input_level = slurp(data, 20);
  meta->output_level = slurp(data, 28);
}

static bool incmem(void *dest, size_t amt, const unsigned char *data, size_t *pos, size_t size) {
  if (*pos + amt > size) return false;
  memcpy(dest, data + *pos, amt);
  *pos += amt;
  return true;
}

static bool inlimit_impl(size_t value, size_t limit, char *name) {
  if (value > limit) {
    // fprintf(stderr, "PLZ make %s into at least %zu\n", name, value);
    return false;
  }
  return true;
}
#define inlimit(value, limit) inlimit_impl(value, limit, #limit)

bool read_layer_array(WaveNetModel *la, const unsigned char* data, size_t size)
{
  int input_size = pull(data, 0);
  int condition_size = pull(data, 2);
  int head_size = pull(data, 4);
  if (input_size != 1 || condition_size != 1 || head_size != 1) return false;
  la-> n_chan = pull(data, 6);
  int bottleneck = pull(data, 8);
  if (bottleneck != la->n_chan) return false;  // NOW you did it
  la-> head_kernel_size = pull(data, 10); // was reserved; stores head_kernel_size
  if (la->head_kernel_size == 0) {
    la->head_kernel_size = 1;
  }

  la-> head_bias = take(data, 12);
  la-> n_layers = take(data, 13);
  if (pull(data, 14) != 1) return false; // groups_input
  if (pull(data, 16) != 1) return false; // groups_input_mixin

  if (take(data, 18) == 0) return false; // layer1x1_active
  if (pull(data, 19) != 1) return false; // layer1x1_groups
  // reserved = take(data, 21)

  if (take(data, 22)) return 0; // head1x1_active
  // int head1x1_out_channels = pull(data, 23);
  // int head1x1_groups = pull(data, 25);
  // reserved = take(data, 27)

  // no FiLM, check it is inactive
  for (int i = 0; i < 8; i++) {
    uint8_t flags = take(data, 28+i*4);
    bool active = (flags & 0x01) != 0;
    if (active) return false;
  }

  size_t pos = 60;
  if (!inlimit(la->n_layers, MAX_LAYERS)) return false;
  if (!incmem(la->dilations, la->n_layers * sizeof(uint32_t), data, &pos, size)) return false;
  if (!incmem(la->kernel_sizes, la->n_layers * sizeof(uint16_t), data, &pos, size)) return false;

  for (size_t i = 0; i < la->n_layers; i++) {
    if (pos + 2 > size) return false;
    int activation_type = take(data, pos);
    const int LeakyReLU = 4;
    if (activation_type != LeakyReLU) return false;
    int activation_n_param = take(data, pos+1);
    if (activation_n_param != 1) return false;
    float param = collect(data, pos+2);
    if (!(0.009 < param && param < 0.011)) return false;
    pos += 6;
  }
  for (size_t i = 0; i < la->n_layers; i++) {
    if (take(data, pos++) != 0) return false;  // gating mode
  }

  for (size_t i = 0; i < la->n_layers; i++) {
    // int activation_type = take(data, pos);
    int activation_n_param = take(data, pos+1);
    if (activation_n_param != 0) return false;
    pos += 2;
  }

  return true;
}

size_t set_offsets(WaveNetModel *la, size_t weight_pos, size_t max_samples)
{
  // upscale block
  la->whpos_upscale = weight_pos;
  weight_pos += 1*la->n_chan;

  int history_offset = 0;

  for (size_t i = 0; i < la->n_layers; i++) {
    // CONV
    int conv_channel_in = la->n_chan;
    int conv_channel_out = la->n_chan;
    int conv_kernel_size = la->kernel_sizes[i];
    // bool conv_bias = true;
    // fprintf(stderr, "le %2zu chanel: in=%d, out=%d, kernal=%d, dilation=%d group_by:%d\n", i, conv_channel_in, conv_channel_out, conv_kernel_size, la->dilations[i], la->groups_input);

    int conv_weights = conv_kernel_size * conv_channel_in * conv_channel_out;
    int conv_bias_weights = conv_channel_out;

    int history_size = ((la->kernel_sizes[i]-1)*la->dilations[i]+ max_samples);
    int roundup_size = 1;
    while (roundup_size < history_size) {
      roundup_size <<= 1;
    }
    la->time_mask[i] = roundup_size - 1;
    la->history_offset[i] = history_offset;
    history_offset += roundup_size*la->n_chan; // n_chan is outside, we never split a block


    la->whpos_layer[i].conv_kernel = weight_pos;
    weight_pos += conv_weights;
    la->whpos_layer[i].conv_bias = weight_pos;
    weight_pos += conv_bias_weights;

    // INPUT MIXIN
    int mixin_in = 1;
    int mixin_out = la->n_chan;
    // bool mixin_bias = false;
    int mixin_weights = mixin_in * mixin_out;
    la->whpos_layer[i].mixin = weight_pos;
    weight_pos += mixin_weights;

    int layer1x1_in = la->n_chan;
    int layer1x1_out = la->n_chan;
    // bool layer1x1_bias = true;

    int layer1x1_weights = layer1x1_in * layer1x1_out;
    la->whpos_layer[i].layer1x1_kernel = weight_pos;
    weight_pos += layer1x1_weights;
    la->whpos_layer[i].layer1x1_bias = weight_pos;
    weight_pos += layer1x1_out;

  }

  int headsum_in = la->n_chan;
  int headsum_out = 1;
  int headsum_kernel_size = la->head_kernel_size;
  bool headsum_bias = la->head_bias;
  int headsum_weights = headsum_in * headsum_out * headsum_kernel_size;
  int headsum_n_bias = (headsum_bias ? headsum_out : 0);

  la->whpos_headsum_kernel = weight_pos;
  weight_pos += headsum_weights;
  la->whpos_headsum_bias = headsum_bias ? weight_pos : 0xFFFFFFFF;
  weight_pos += headsum_n_bias;

  int headsum_size_raw = (la->head_kernel_size -1 + max_samples);
  int headsum_size = 1;
  while (headsum_size < headsum_size_raw) {
    headsum_size <<= 1;
  }
  // headsum history is bottleneck of all layers (stored where last layer would be)
  la->history_offset[la->n_layers] = history_offset;
  history_offset += headsum_size*la->n_chan;
  la->time_mask[la->n_layers] = headsum_size - 1;

#ifdef ACTIVATION_HISTORY
  memset(la->activation_history, 0, sizeof la->activation_history);
#endif

  la->history_size = history_offset;
  return weight_pos;
}

bool read_model(WaveNetModel *la, const unsigned char* data, size_t size, size_t max_samples)
{
  if (size < FILE_HEADER_SIZE + METADATA_BLOCK_SIZE + 4) {
    return false;
  }

  uint32_t magic = boke(data, 0);
  uint16_t version = pull(data, 4);
  // uint16_t flags = pull(data, 6);
  uint32_t total_file_size = boke(data, 8);
  uint32_t weights_offset = boke(data, 12);
  uint32_t total_weight_count = boke(data, 16);
  // uint32_t model_block_size = boke(data, 20);
  uint32_t stored_checksum = boke(data, 24);
  // reserved header bytes..

  if (magic != NAMB_MAGIC || version != NAMB_FORMAT_VERSION) {
    return false;
  }
  size_t expected_weights_end = weights_offset + total_weight_count * sizeof(float);
  if (size < total_file_size || total_file_size < expected_weights_end) {
    return false;
  }
  (void)stored_checksum; // haii

  read_metadata(&la->metadata, data + FILE_HEADER_SIZE);
  
  const int MODEL_HEADER_OFFSET = FILE_HEADER_SIZE + METADATA_BLOCK_SIZE;
  uint8_t arch = take(data, MODEL_HEADER_OFFSET);
  if (arch != ARCH_WAVENET) {
    return false;
  }
  const int MODEL_CONFIG_OFFSET = MODEL_HEADER_OFFSET + 4;
  const int WAVENET_CONFIG_SIZE = 4;

  if (MODEL_CONFIG_OFFSET + WAVENET_CONFIG_SIZE > total_file_size) {
    return false;
  }

  int in_channels = take(data, MODEL_CONFIG_OFFSET);
  bool has_head = take(data, MODEL_CONFIG_OFFSET + 1);
  uint8_t num_layer_arrays = take(data, MODEL_CONFIG_OFFSET + 2);
  uint8_t has_condition_dsp = take(data, MODEL_CONFIG_OFFSET + 3);
  if (in_channels != 1 || has_head || has_condition_dsp || num_layer_arrays != 1) return false;

  int layer_off = MODEL_CONFIG_OFFSET + 4;
  if (!read_layer_array(la, data + layer_off, total_file_size - layer_off)) {
    return false;
  }

  size_t weight_pos = 0;
  weight_pos = set_offsets(la, weight_pos, max_samples);

  uint32_t whpos_head_scale = weight_pos;
  weight_pos += 1;
  if (weight_pos != total_weight_count) return false;

  if (total_weight_count > MAX_WEIGHT_COUNT) return false;
  memcpy(la->weights, data+weights_offset, 4*total_weight_count);
#ifdef FIPPLA
  size_t c = la->n_chan;
  for (size_t i = 0; i < la->n_layers; i++) {
    uint32_t kernel_size = la->kernel_sizes[i];
    const size_t off = la->whpos_layer[i].conv_kernel;
    const unsigned char *src = data + weights_offset + 4*off;
    float *wh_kernel = la->weights + off;
    for (size_t o = 0; o < kernel_size; o++) {
      for (size_t k = 0; k < c; k++) {
        for (size_t m = 0; m < c; m++) {
          memcpy(&wh_kernel[c*c*o + c*k +m] , &src[4*(kernel_size*(k*c+m)+o)], 4);
        }
      }
    }
  }
#endif

  la->head_scale = la->weights[whpos_head_scale];

  int prewarm_len = 1;
  for (size_t i = 0; i < la->n_layers; i++) {
    prewarm_len += (la->dilations[i]*la->kernel_sizes[i]-1);
  }
  prewarm_len += la->head_kernel_size - 1;

  la->prewarm_len = prewarm_len;

  return true;
}

