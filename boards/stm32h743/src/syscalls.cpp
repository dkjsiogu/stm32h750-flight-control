#include <cerrno>
#include <cstddef>
#include <sys/stat.h>

extern "C" {

/** 链接脚本提供的堆起始地址。 */
extern char _heap_start;
/** 链接脚本提供的堆结束地址。 */
extern char _heap_end;

/**
 * newlib 堆扩展入口。
 *
 * @param increment 本次申请的堆增长字节数。
 * @return 成功时返回增长前的堆指针，失败时返回 -1。
 */
void* _sbrk(std::ptrdiff_t increment) {
    /** 当前堆顶指针，只能在单核启动后的固件上下文中使用。 */
    static char* heap_current = &_heap_start;
    /** 本次申请前的堆顶，用作返回值。 */
    char* previous_heap = heap_current;
    /** 本次申请后的堆顶，用于检查是否越界。 */
    char* next_heap = heap_current + increment;

    if (next_heap > &_heap_end) {
        errno = ENOMEM;
        return reinterpret_cast<void*>(-1);
    }

    heap_current = next_heap;
    return previous_heap;
}

/**
 * 进程号查询桩函数。
 *
 * @return 固件环境下固定返回 1。
 */
int _getpid() {
    return 1;
}

/**
 * 信号发送桩函数。
 *
 * @param pid 目标进程号，裸机环境不使用。
 * @param sig 信号编号，裸机环境不使用。
 * @return 固件环境下固定失败。
 */
int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

/**
 * 文件关闭桩函数。
 *
 * @param file 文件描述符，裸机环境不使用。
 * @return 固件环境下固定失败。
 */
int _close(int file) {
    (void)file;
    return -1;
}

/**
 * 文件状态查询桩函数。
 *
 * @param file 文件描述符，裸机环境不使用。
 * @param status 输出文件状态，按字符设备处理。
 * @return 成功返回 0。
 */
int _fstat(int file, struct stat* status) {
    (void)file;
    if (status) {
        status->st_mode = S_IFCHR;
    }
    return 0;
}

/**
 * 终端判断桩函数。
 *
 * @param file 文件描述符，裸机环境不使用。
 * @return 固件日志流按终端处理。
 */
int _isatty(int file) {
    (void)file;
    return 1;
}

/**
 * 文件偏移设置桩函数。
 *
 * @param file 文件描述符，裸机环境不使用。
 * @param offset 偏移量，裸机环境不使用。
 * @param origin 偏移起点，裸机环境不使用。
 * @return 固件环境下固定返回 0。
 */
int _lseek(int file, int offset, int origin) {
    (void)file;
    (void)offset;
    (void)origin;
    return 0;
}

/**
 * 文件读取桩函数。
 *
 * @param file 文件描述符，裸机环境不使用。
 * @param buffer 输出缓冲区，裸机环境不使用。
 * @param length 请求读取长度，裸机环境不使用。
 * @return 固件环境下固定返回 0。
 */
int _read(int file, char* buffer, int length) {
    (void)file;
    (void)buffer;
    (void)length;
    return 0;
}

/**
 * 文件写入桩函数。
 *
 * @param file 文件描述符，裸机环境不使用。
 * @param buffer 输入缓冲区，裸机环境不使用。
 * @param length 请求写入长度。
 * @return 返回 length，表示日志字节已被丢弃处理。
 */
int _write(int file, char* buffer, int length) {
    (void)file;
    (void)buffer;
    return length;
}

}  // extern "C"
