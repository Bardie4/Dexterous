// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_FINGERBROADCAST_QUAD_DOUBLE_ME_H_
#define FLATBUFFERS_GENERATED_FINGERBROADCAST_QUAD_DOUBLE_ME_H_

#include "flatbuffers/flatbuffers.h"

namespace quad_double_me {

struct HandBroadcast;

struct FingerStates;

struct HandBroadcast FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_HAND = 4
  };
  const flatbuffers::Vector<flatbuffers::Offset<FingerStates>> *hand() const {
    return GetPointer<const flatbuffers::Vector<flatbuffers::Offset<FingerStates>> *>(VT_HAND);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_HAND) &&
           verifier.VerifyVector(hand()) &&
           verifier.VerifyVectorOfTables(hand()) &&
           verifier.EndTable();
  }
};

struct HandBroadcastBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_hand(flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<FingerStates>>> hand) {
    fbb_.AddOffset(HandBroadcast::VT_HAND, hand);
  }
  explicit HandBroadcastBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  HandBroadcastBuilder &operator=(const HandBroadcastBuilder &);
  flatbuffers::Offset<HandBroadcast> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<HandBroadcast>(end);
    return o;
  }
};

inline flatbuffers::Offset<HandBroadcast> CreateHandBroadcast(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<FingerStates>>> hand = 0) {
  HandBroadcastBuilder builder_(_fbb);
  builder_.add_hand(hand);
  return builder_.Finish();
}

inline flatbuffers::Offset<HandBroadcast> CreateHandBroadcastDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<flatbuffers::Offset<FingerStates>> *hand = nullptr) {
  auto hand__ = hand ? _fbb.CreateVector<flatbuffers::Offset<FingerStates>>(*hand) : 0;
  return quad_double_me::CreateHandBroadcast(
      _fbb,
      hand__);
}

struct FingerStates FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_ID = 4,
    VT_ANGLE1 = 6,
    VT_ANGLE2 = 8,
    VT_ANGVELOCITY1 = 10,
    VT_ANGVELOCITY2 = 12,
    VT_ANGACCELERATION1 = 14,
    VT_ANGACCELERATION2 = 16,
    VT_COMMANDEDTORQUE1 = 18,
    VT_COMMANDEDTORQUE2 = 20,
    VT_EMPTY1 = 22,
    VT_EMPTY2 = 24,
    VT_EMPTY3 = 26,
    VT_EMPTY4 = 28
  };
  int16_t id() const {
    return GetField<int16_t>(VT_ID, 0);
  }
  float angle1() const {
    return GetField<float>(VT_ANGLE1, 0.0f);
  }
  float angle2() const {
    return GetField<float>(VT_ANGLE2, 0.0f);
  }
  float angVelocity1() const {
    return GetField<float>(VT_ANGVELOCITY1, 0.0f);
  }
  float angVelocity2() const {
    return GetField<float>(VT_ANGVELOCITY2, 0.0f);
  }
  float angAcceleration1() const {
    return GetField<float>(VT_ANGACCELERATION1, 0.0f);
  }
  float angAcceleration2() const {
    return GetField<float>(VT_ANGACCELERATION2, 0.0f);
  }
  float commandedTorque1() const {
    return GetField<float>(VT_COMMANDEDTORQUE1, 0.0f);
  }
  float commandedTorque2() const {
    return GetField<float>(VT_COMMANDEDTORQUE2, 0.0f);
  }
  float Empty1() const {
    return GetField<float>(VT_EMPTY1, 0.0f);
  }
  float empty2() const {
    return GetField<float>(VT_EMPTY2, 0.0f);
  }
  float empty3() const {
    return GetField<float>(VT_EMPTY3, 0.0f);
  }
  float empty4() const {
    return GetField<float>(VT_EMPTY4, 0.0f);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyField<int16_t>(verifier, VT_ID) &&
           VerifyField<float>(verifier, VT_ANGLE1) &&
           VerifyField<float>(verifier, VT_ANGLE2) &&
           VerifyField<float>(verifier, VT_ANGVELOCITY1) &&
           VerifyField<float>(verifier, VT_ANGVELOCITY2) &&
           VerifyField<float>(verifier, VT_ANGACCELERATION1) &&
           VerifyField<float>(verifier, VT_ANGACCELERATION2) &&
           VerifyField<float>(verifier, VT_COMMANDEDTORQUE1) &&
           VerifyField<float>(verifier, VT_COMMANDEDTORQUE2) &&
           VerifyField<float>(verifier, VT_EMPTY1) &&
           VerifyField<float>(verifier, VT_EMPTY2) &&
           VerifyField<float>(verifier, VT_EMPTY3) &&
           VerifyField<float>(verifier, VT_EMPTY4) &&
           verifier.EndTable();
  }
};

struct FingerStatesBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_id(int16_t id) {
    fbb_.AddElement<int16_t>(FingerStates::VT_ID, id, 0);
  }
  void add_angle1(float angle1) {
    fbb_.AddElement<float>(FingerStates::VT_ANGLE1, angle1, 0.0f);
  }
  void add_angle2(float angle2) {
    fbb_.AddElement<float>(FingerStates::VT_ANGLE2, angle2, 0.0f);
  }
  void add_angVelocity1(float angVelocity1) {
    fbb_.AddElement<float>(FingerStates::VT_ANGVELOCITY1, angVelocity1, 0.0f);
  }
  void add_angVelocity2(float angVelocity2) {
    fbb_.AddElement<float>(FingerStates::VT_ANGVELOCITY2, angVelocity2, 0.0f);
  }
  void add_angAcceleration1(float angAcceleration1) {
    fbb_.AddElement<float>(FingerStates::VT_ANGACCELERATION1, angAcceleration1, 0.0f);
  }
  void add_angAcceleration2(float angAcceleration2) {
    fbb_.AddElement<float>(FingerStates::VT_ANGACCELERATION2, angAcceleration2, 0.0f);
  }
  void add_commandedTorque1(float commandedTorque1) {
    fbb_.AddElement<float>(FingerStates::VT_COMMANDEDTORQUE1, commandedTorque1, 0.0f);
  }
  void add_commandedTorque2(float commandedTorque2) {
    fbb_.AddElement<float>(FingerStates::VT_COMMANDEDTORQUE2, commandedTorque2, 0.0f);
  }
  void add_Empty1(float Empty1) {
    fbb_.AddElement<float>(FingerStates::VT_EMPTY1, Empty1, 0.0f);
  }
  void add_empty2(float empty2) {
    fbb_.AddElement<float>(FingerStates::VT_EMPTY2, empty2, 0.0f);
  }
  void add_empty3(float empty3) {
    fbb_.AddElement<float>(FingerStates::VT_EMPTY3, empty3, 0.0f);
  }
  void add_empty4(float empty4) {
    fbb_.AddElement<float>(FingerStates::VT_EMPTY4, empty4, 0.0f);
  }
  explicit FingerStatesBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  FingerStatesBuilder &operator=(const FingerStatesBuilder &);
  flatbuffers::Offset<FingerStates> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<FingerStates>(end);
    return o;
  }
};

inline flatbuffers::Offset<FingerStates> CreateFingerStates(
    flatbuffers::FlatBufferBuilder &_fbb,
    int16_t id = 0,
    float angle1 = 0.0f,
    float angle2 = 0.0f,
    float angVelocity1 = 0.0f,
    float angVelocity2 = 0.0f,
    float angAcceleration1 = 0.0f,
    float angAcceleration2 = 0.0f,
    float commandedTorque1 = 0.0f,
    float commandedTorque2 = 0.0f,
    float Empty1 = 0.0f,
    float empty2 = 0.0f,
    float empty3 = 0.0f,
    float empty4 = 0.0f) {
  FingerStatesBuilder builder_(_fbb);
  builder_.add_empty4(empty4);
  builder_.add_empty3(empty3);
  builder_.add_empty2(empty2);
  builder_.add_Empty1(Empty1);
  builder_.add_commandedTorque2(commandedTorque2);
  builder_.add_commandedTorque1(commandedTorque1);
  builder_.add_angAcceleration2(angAcceleration2);
  builder_.add_angAcceleration1(angAcceleration1);
  builder_.add_angVelocity2(angVelocity2);
  builder_.add_angVelocity1(angVelocity1);
  builder_.add_angle2(angle2);
  builder_.add_angle1(angle1);
  builder_.add_id(id);
  return builder_.Finish();
}

inline const quad_double_me::HandBroadcast *GetHandBroadcast(const void *buf) {
  return flatbuffers::GetRoot<quad_double_me::HandBroadcast>(buf);
}

inline const quad_double_me::HandBroadcast *GetSizePrefixedHandBroadcast(const void *buf) {
  return flatbuffers::GetSizePrefixedRoot<quad_double_me::HandBroadcast>(buf);
}

inline const char *HandBroadcastIdentifier() {
  return "HBRC";
}

inline bool HandBroadcastBufferHasIdentifier(const void *buf) {
  return flatbuffers::BufferHasIdentifier(
      buf, HandBroadcastIdentifier());
}

inline bool VerifyHandBroadcastBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<quad_double_me::HandBroadcast>(HandBroadcastIdentifier());
}

inline bool VerifySizePrefixedHandBroadcastBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<quad_double_me::HandBroadcast>(HandBroadcastIdentifier());
}

inline void FinishHandBroadcastBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<quad_double_me::HandBroadcast> root) {
  fbb.Finish(root, HandBroadcastIdentifier());
}

inline void FinishSizePrefixedHandBroadcastBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<quad_double_me::HandBroadcast> root) {
  fbb.FinishSizePrefixed(root, HandBroadcastIdentifier());
}

}  // namespace quad_double_me

#endif  // FLATBUFFERS_GENERATED_FINGERBROADCAST_QUAD_DOUBLE_ME_H_
