
#include <boost/asio.hpp>
#include "NIST/RCSMsgQueue.h"

namespace asio = boost::asio;
extern RCS::CMessageQueue<std::string> incmds;

struct socket_line_connection {
    boost::asio::io_service io_service;
    int _portnum;
    boost::shared_ptr<asio::ip::tcp::socket> pSocket;
    boost::asio::streambuf buf_;

    void Init(int portnum) {
        _portnum = portnum;
        asio::ip::tcp::acceptor acc(
                io_service,
                asio::ip::tcp::endpoint(asio::ip::tcp::v4(), _portnum));
        pSocket = boost::shared_ptr<asio::ip::tcp::socket>(new asio::ip::tcp::socket(io_service));
        acc.accept(*pSocket);
        start();
    }

    void start() {
        boost::asio::async_read_until(*pSocket, buf_, "\n",
                boost::bind(&socket_line_connection::readline, this, _1,_2));
    }

    void readline(boost::system::error_code ec, size_t n) {
        if ((boost::asio::error::eof == ec) ||
                (boost::asio::error::connection_reset == ec)) {
            boost::system::error_code error;
            pSocket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
            pSocket->close(error);
            Init(_portnum); // disconnected start again
        } else {
            std::string data = asio::buffer_cast<const char*>(buf_.data());
            data.resize(n);
            std::cout << "Add: " << data; // debug print
            incmds.AddMsgQueue(data);
            buf_.consume(n); // will infinite loop here unless consumed
            start();
        }
    }

    void sendline(std::string str) {
        // Write data to server that contains a delimiter.
        try {
            pSocket->send(boost::asio::buffer(str, str.size()));
        } catch (boost::system::system_error err) {
            std::cerr << boost::diagnostic_information(err);
        } catch (...) {
            std::cerr << "Aborted SyncWrite\n";
        }
    }


};
