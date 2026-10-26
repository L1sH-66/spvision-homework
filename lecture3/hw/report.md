# 项目理解报告

以下解释对应本目录实现，使用 AI 协助编写。运行和验收方法见 README.md。

## 1. 图像生命周期与所有权

ImageSequenceSource 用 buffer_ 模拟相机的循环缓冲区。下一次 raw.copyTo(buffer_) 在尺寸、类型一致时复用内存。原来的 frame.image = buffer_ 只复制 Mat 头部，共享像素；引用计数只能延长内存寿命，不能阻止像素被下一帧覆盖。

修改为 buffer_.clone() 后，每帧获得独立像素。生产线程将 Frame 移入队列，消费者从队列移出；源对象可以继续复用 buffer_，已经入队的数据不会变化。std::move 只转移资源，不能代替最初的深复制。

## 2. 并发处理与恰好一次

保留题目 BlockingQueue。push 在互斥锁内追加元素，pop 在同一把锁内取出并删除队首，因此一个元素只能被一个 worker 取得。多个 worker 并行处理各自的 Frame，生产者不处理图像。

生产者读完后 close，notify_all 唤醒等待者。close 不清空已有元素，pop 仍会取出剩余元素；只有关闭且为空才返回 false。正在处理的 worker 保存当前帧后继续 pop，最终退出。wait 先等待生产者完成，再等待所有 worker，避免提前关闭而漏掉后续入队数据。正常输入和可写输出条件下，每个帧 ID 对应唯一文件，恰好处理保存一次。

异常输入或写入失败时不能承诺保存成功，因此线程记录首个异常，继续排空队列，wait 回收线程后重新抛出异常，由 main 报错并返回非零；没有删除 checksum 检查或隐藏处理失败。

## 3. 共享统计数据

生产者更新 produced，多个 worker 更新 processed、saved、corrupted，调用者可读取 snapshot。原来的读旧值、延迟、写回是非原子的读改写，多个线程会丢失更新，且构成 C++ 数据竞争。

现在每次计数更新和整个 snapshot 都由 Statistics 内同一把 mutex 保护；snapshot 是 const，因此 mutex 声明为 mutable。RAII lock_guard 在离开作用域时解锁。所有字段组成的快照在同一个临界区读取，不会和其他更新交错。处理中快照表示那个时刻的进度，完成后各计数满足验收关系。

## 4. 线程关闭协议

显式路径：start 启动至少两个 worker 和一个 producer；producer 读完或捕获异常后关闭队列；wait 先 join producer，再 join workers。全部线程退出后才返回或报告异常，因此资源不会在线程仍访问时析构。

析构路径：析构函数调用同一个 joinThreads，等待生产者完成、队列排空和 worker 退出。析构不重新抛出后台任务异常；需要获知错误的调用者应显式调用 wait。成员在线程回收后才销毁，没有 joinable 的 thread 留给析构触发 terminate，也没有 detach 造成悬空 this。

start 只允许调用一次，重复调用抛 logic_error；wait 可重复调用，每次只 join 仍 joinable 的线程。未 start 时 wait 或析构立即返回；wait 不会关闭未启动对象的队列。生命周期方法由拥有对象的单一线程串行调用，不能与析构并发。线程创建部分失败时关闭队列、回收已创建线程，再向调用者抛出异常。输入源必须最终返回，不能无限阻塞；本题有限文件序列符合条件。
