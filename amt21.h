#ifndef AMT21H_INCLUDED
#define AMT21H_INCLUDED

#include <mstd_atomic>

namespace amt21 {

struct AsyncSerial : public SerialBase
{
	AsyncSerial(PinName tx, PinName rx, int baud)
		: SerialBase(tx, rx, baud)
	{
	}

	void putc(char c)
	{
		this->_base_putc(c);
	}

	char getc() {
		return this->_base_getc();
	}

};

class RS485LineDriver {
public:
	RS485LineDriver(PinName tx, PinName rx, int baud, PinName de)
		: ser_(tx, rx, baud), de_(de)
	{
	}

	template <std::size_t M, std::size_t N>
	void send_and_recv_async(const std::uint8_t (&w)[M], std::uint8_t (&r)[N], Callback<void()> on_recv = nullptr) {
		rx_complete_ = 0;
		on_recv_ = on_recv;

		de_.write(1);

		ser_.write(w, sizeof w, [this](int) { de_.write(0); }, SERIAL_EVENT_TX_COMPLETE);
		ser_.read(r, sizeof r, [this](int) { rx_complete_.store(1); if (on_recv_) { on_recv_(); on_recv_ = nullptr; }; }, SERIAL_EVENT_RX_COMPLETE);
	}

	template <std::size_t M, std::size_t N>
	void send_and_recv(const std::uint8_t (&w)[M], std::uint8_t (&r)[N]) {
		send_and_recv_async(w, r);
		wait_recv();
	}

	void wait_recv() {
		while (!rx_complete_)
			;
	}

private:
	AsyncSerial ser_;
	DigitalOut de_;
	volatile mstd::atomic<bool> rx_complete_;
	Callback<void()> on_recv_;
};

namespace details {

template <int N, class T>
constexpr unsigned int nth_bit(const T &x) {
	static_assert(0 <= N && N < std::numeric_limits<T>::digits, "bad N");
	return (x >> N) & 1;
}

}

std::uint8_t amt21_response_parity(std::uint16_t full) {
	return (full & 0xC000) >> 14;
}

std::uint16_t amt21_response_data(std::uint16_t full) {
	return full & ~(0xC000);
}

inline bool amt21_check_parity(std::uint16_t full) {
	using details::nth_bit;

	bool odd =
		nth_bit<15>(full) ==
			!(nth_bit<13>(full)
			^ nth_bit<11>(full)
			^ nth_bit<9>(full)
			^ nth_bit<7>(full)
			^ nth_bit<5>(full)
			^ nth_bit<3>(full)
			^ nth_bit<1>(full));
	bool even =
		nth_bit<14>(full) ==
			!(nth_bit<12>(full)
			^ nth_bit<10>(full)
			^ nth_bit<8>(full)
			^ nth_bit<6>(full)
			^ nth_bit<4>(full)
			^ nth_bit<2>(full)
			^ nth_bit<0>(full));

	return odd && even;
}

}

#endif
