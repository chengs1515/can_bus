#pragma once

#include <string>

/**
 * @namespace apollo::drivers::canbus
 * @brief apollo::drivers::canbus
 */
namespace can_bus{

/**
 * @class Byte
 * @brief The class of one byte, which is 8 bits.
 *        It includes some operations on one byte.
 */
class Byte {
 public:
  /**
   * @brief Constructor which takes a pointer to a one-byte unsigned integer.
   * @param value The pointer to a one-byte unsigned integer for construction.
   */
  explicit Byte(const uint8_t *value);

  /**
   * @brief Constructor which takes a reference to a one-byte unsigned integer.
   * @param value The reference to a one-byte unsigned integer for construction.
   */
  Byte(const Byte &value);

  /**
   * @brief Desctructor.
   */
  ~Byte() = default;

  /**
   * @brief Transform an integer with the size of one byte to its hexadecimal
   *        represented by a string.
   * @param value The target integer to transform.
   * @return Hexadecimal representing the target integer.
   */
  static std::string byte_to_hex(const uint8_t value);

  /**
   * @brief Transform an integer with the size of 4 bytes to its hexadecimal
   *        represented by a string.
   * @param value The target integer to transform.
   * @return Hexadecimal representing the target integer.
   */
  static std::string byte_to_hex(const uint32_t value);

  /**
   * @brief Transform an integer with the size of one byte to its binary
   *        represented by a string.
   * @param value The target integer to transform.
   * @return Binary representing the target integer.
   */
  static std::string byte_to_binary(const uint8_t value);

  /**
   * @brief Set the bit on a specified position to one.
   * @param pos The position of the bit to be set to one.
   */
  void set_bit_1(const int32_t pos);

  /**
   * @brief Set the bit on a specified position to zero.
   * @param pos The position of the bit to be set to zero.
   */
  void set_bit_0(const int32_t pos);

  /**
   * @brief Check if the bit on a specified position is one.
   * @param pos The position of the bit to check.
   * @return If the bit on a specified position is one.
   */
  bool is_bit_1(const int32_t pos) const;

  /**
   * @brief Reset this Byte by a specified one-byte unsigned integer.
   * @param value The one-byte unsigned integer to set this Byte.
   */
  void set_value(const uint8_t value);

  /**
   * @brief Reset the higher 4 bits as the higher 4 bits of a specified one-byte
   *        unsigned integer.
   * @param value The one-byte unsigned integer whose higher 4 bits are used to
   *        set this Byte's higher 4 bits.
   */
  void set_value_high_4_bits(const uint8_t value);

  /**
   * @brief Reset the lower 4 bits as the lower 4 bits of a specified one-byte
   *        unsigned integer.
   * @param value The one-byte unsigned integer whose lower 4 bits are used to
   *        set this Byte's lower 4 bits.
   */
  void set_value_low_4_bits(const uint8_t value);

  /**
   * @brief Reset some consecutive bits starting from a specified position with
   *        a certain length of another one-byte unsigned integer.
   * @param value The one-byte unsigned integer whose certain bits are used
   *        to set this Byte.
   * @param start_pos The starting position (from the lowest) of the bits.
   * @param length The length of the consecutive bits.
   */
  void set_value(const uint8_t value, const int32_t start_pos,
                 const int32_t length);

  /**
   * @brief Get the one-byte unsigned integer.
   * @return The one-byte unsigned integer.
   */
  uint8_t get_byte() const;

  /**
   * @brief Get a one-byte unsigned integer representing the higher 4 bits.
   * @return The one-byte unsigned integer representing the higher 4 bits.
   */
  uint8_t get_byte_high_4_bits() const;

  /**
   * @brief Get a one-byte unsigned integer representing the lower 4 bits.
   * @return The one-byte unsigned integer representing the lower 4 bits.
   */
  uint8_t get_byte_low_4_bits() const;

  /**
   * @brief Get a one-byte unsigned integer representing the consecutive bits
   *        from a specified position (from lowest) by a certain length.
   * @param start_pos The starting position (from lowest) of bits.
   * @param length The length of the selected consecutive bits.
   * @return The one-byte unsigned integer representing the selected bits.
   */
  uint8_t get_byte(const int32_t start_pos, const int32_t length) const;

  /**
   * @brief Transform to its hexadecimal represented by a string.
   * @return Hexadecimal representing the Byte.
   */
  std::string to_hex_string() const;

  /**
   * @brief Transform to its binary represented by a string.
   * @return Binary representing the Byte.
   */
  std::string to_binary_string() const;

 private:
  uint8_t *value_;
};

}  // namespace canbus
