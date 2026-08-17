# Third-party notices

## lora-rs STM32WL interface example

Parts of `src/radio.rs` are adapted from the STM32WL interface example in
`lora-rs`, tag `lora-phy-v3.0.1`, file `examples/stm32wl/src/iv.rs`:

<https://github.com/lora-rs/lora-rs/blob/lora-phy-v3.0.1/examples/stm32wl/src/iv.rs>

The upstream work is offered under `MIT OR Apache-2.0`. This project uses it
under the MIT terms below.

> Copyright (c) 2022-2023 lora-phy project contributors
>
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in
> all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.

The `lora-phy` crate itself remains under its upstream `MIT OR Apache-2.0`
license. RAK3172-T electrical and RF configuration constants were checked
against RAKwireless's official `WisDuo_RAK3172-T_Board` support files; those
facts are not a copy of the RAK implementation.
