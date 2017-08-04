#include <cstring>
#include <cerrno>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <fstream>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <tclap/CmdLine.h>

namespace serial {
	enum class BaudRate : speed_t {
		_0 = B0,
		_50 = B50,
		_75 = B75,
		_110 = B110,
		_134 = B134,
		_150 = B150,
		_200 = B200,
		_300 = B300,
		_600 = B600,
		_1200 = B1200,
		_1800 = B1800,
		_2400 = B2400,
		_4800 = B4800,
		_9600 = B9600,
		_19200 = B19200,
		_38400 = B38400,
		_57600 = B57600,
		_115200 = B115200,
		_230400 = B230400
	};

	class File {
		int file_id_;
		bool is_open_;
		BaudRate iospeed_;

		void update_baud_rate() {
			if (!is_open_) {
				return;
			}

			termios attributes;

			if (tcgetattr(file_id_, &attributes) != 0) {
				std::string message = std::strerror(errno);

				std::ostringstream oss;
				oss
					<< "serial::File::update_baud_rate: "
					<< "error getting termios attributes: "
					<< message;

				throw std::runtime_error(oss.str());
			}

			#if defined(_DEFAULT_SOURCE) || defined(_BSD_SOURCE)

			if (cfsetspeed(&attributes, static_cast<speed_t>(iospeed_)) != 0) {
				std::string message = std::strerror(errno);

				std::ostringstream oss;
				oss
					<< "serial::File::update_baud_rate: "
					<< "error setting speed: "
					<< message;

				throw std::runtime_error(oss.str());
			}

			#else

			if (
				cfsetispeed(&attributes, static_cast<speed_t>(iospeed_)) != 0
			) {
				std::string message = std::strerror(errno);

				std::ostringstream oss;
				oss
					<< "serial::File::update_baud_rate: "
					<< "error setting input speed: "
					<< message;

				throw std::runtime_error(oss.str());
			}

			if (
				cfsetospeed(&attributes, static_cast<speed_t>(iospeed_)) != 0
			) {
				std::string message = std::strerror(errno);

				std::ostringstream oss;
				oss
					<< "serial::File::update_baud_rate: "
					<< "error setting output speed: "
					<< message;

				throw std::runtime_error(oss.str());
			}

			#endif

			if (tcsetattr(file_id_, TCSANOW, &attributes) != 0) {
				std::string message = std::strerror(errno);

				std::ostringstream oss;
				oss
					<< "serial::File::update_baud_rate: "
					<< "error setting termios attributes: "
					<< message;

				throw std::runtime_error(oss.str());
			}
		}

	public:
		File() : file_id_(-1), is_open_(false), iospeed_(BaudRate::_9600) { }

		File(
			const std::string &path,
			int flags,
			BaudRate iospeed = BaudRate::_9600
		) :
			file_id_(-1),
			is_open_(false),
			iospeed_(iospeed)
		{
			open(path, flags);
			update_baud_rate();
		}

		~File() {
			close();
		}

		void open(const std::string &path, int flags) {
			if (is_open_) {
				close();
			}

			file_id_ = ::open(path.c_str(), flags);

			if (file_id_ == -1) {
				is_open_ = false;

				std::string message = std::strerror(errno);

				std::ostringstream oss;
				oss
					<< "serial::File::open: error opening path \"" << path
					<< "\": " << message;

				throw std::runtime_error(oss.str());
			}

			is_open_ = true;
		}

		void close() {
			if (!is_open_) {
				return;
			}

			if (::close(file_id_) != 0) {
				std::string message = std::strerror(errno);

				std::ostringstream oss;
				oss
					<< "serial::File::close: error closing file: " << message;

				throw std::runtime_error(oss.str());
			}

			is_open_ = false;	
		}

		std::ptrdiff_t write(const std::vector<unsigned char> &buffer) {
			if (!is_open_) {
				throw std::runtime_error("serial::File::write: file not open");
			}

			ssize_t bytes_written = ::write(
				file_id_,
				buffer.data(),
				buffer.size()
			);

			if (bytes_written == -1) {
				std::string message = std::strerror(errno);

				std::ostringstream oss;
				oss
					<< "serial::File::write: error writing to file: "
					<< message;

				throw std::runtime_error(oss.str());
			}

			return bytes_written;
		}

		std::ptrdiff_t write(unsigned char byte) {
			return write(std::vector<unsigned char>({ byte }));
		}

		std::ptrdiff_t write(const unsigned char *buffer, std::size_t size) {
			return write(std::vector<unsigned char>(buffer, buffer + size));
		}

		template <std::size_t N>
		std::ptrdiff_t write(const unsigned char (&buffer)[N]) {
			return write(buffer, N);
		}

		std::vector<unsigned char> read(std::size_t count) {
			std::vector<unsigned char> buffer(count);

			ssize_t bytes_read = ::read(file_id_, buffer.data(), count);

			if (bytes_read == -1) {
				std::string message = std::strerror(errno);

				std::ostringstream oss;
				oss
					<< "serial::File::read: error reading from file: "
					<< message;

				throw std::runtime_error(oss.str());
			}

			buffer.resize(bytes_read);
			return buffer;
		}

		unsigned char read() {
			return read(1).front();
		}
	};
}

namespace time_util {
	template <typename T>
	void wait(const std::chrono::duration<T> &duration) {
		auto start = std::chrono::high_resolution_clock::now();
		auto now = std::chrono::high_resolution_clock::now();

		while (now - start < duration) {
			now = std::chrono::high_resolution_clock::now();
		}
	}

	template <typename T>
	void wait(const T &duration) {
		wait(std::chrono::duration<T>(duration));
	}
}

template <typename T>
std::ostream& operator<<(std::ostream &os, const std::vector<T> &x) {
	if (x.empty()) {
		return os << "{ }";
	}

	auto first = x.cbegin();
	auto last = x.cend();

	os << "{ " << *first++;

	for (; first != last; ++first) {
		os << ", " << *first;
	}

	return os << " }";
}

void listen(serial::File &file) {
	std::vector<unsigned char> response;

	while (true) {
		try {
			response = file.read(2048);

			if (!response.empty()) {
				std::cout << "response = " << response << std::endl;
			}
		} catch (...) { }
	}
}

int main(int argc, char *argv[]) {
	std::string path;

	try {
		TCLAP::CmdLine cmd("serial communication test", ' ', "0.1.0");

		TCLAP::UnlabeledValueArg<std::string> path_arg(
			"pathname",
			"path to file",
			false,
			"/dev/ttyUSB0",
			"string"
		);

		cmd.add(path_arg);
		cmd.parse(argc, argv);

		path = path_arg.getValue();
	} catch (const TCLAP::ArgException &e) {
		std::cerr
			<< "error: " << e.error() << " for arg " << e.argId() << std::endl;
	}

	std::cout << "opening file at path " << path << "..." << std::endl;

	serial::File device(
		path,
		O_RDWR | O_NDELAY,
		serial::BaudRate::_115200
	);

	std::cout << "opened file successfully" << std::endl;

	time_util::wait(1.0);

	std::thread listener_thread(listen, std::ref(device));

	while (true) {
		device.write(0x0f);
		time_util::wait(1.0);
	}

	listener_thread.join();
}