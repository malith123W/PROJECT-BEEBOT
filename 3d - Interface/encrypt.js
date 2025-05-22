// encrypt.js
import { ModeOfOperation, utils } from 'aes-js';

const key = [139, 190, 60, 202, 65, 17, 46, 241, 173, 243, 170, 221, 190, 150, 115, 6];

export function decrypt(data) {
    try {
        const iv = data.slice(0, 16);
        const cipher = data.slice(16);
        const aesOfb = new ModeOfOperation.ofb(key, iv);
        const decryptedBytes = aesOfb.decrypt(cipher);
        return decryptedBytes;
    } catch (e) {
        console.error('decryption error', e);
        return null;
    }
}

export function encryptString(data) {
    try {
        const iv = new Uint8Array([21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36]);
        const aesOfb = new ModeOfOperation.ofb(key, iv);
        const bytes = utils.utf8.toBytes(data);
        const encryptedBytes = aesOfb.encrypt(bytes);

        const concat = new Uint8Array(iv.length + encryptedBytes.length);
        concat.set(iv);
        concat.set(encryptedBytes, iv.length);
        return concat;
    } catch (e) {
        console.error('encryption error', e);
        return null;
    }
}

export function decryptString(data) {
    const decryptedBytes = decrypt(data);
    return decryptedBytes ? new TextDecoder().decode(decryptedBytes) : null;
}
